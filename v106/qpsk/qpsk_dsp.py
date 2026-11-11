"""QPSK 基带 DSP：成型、捕获、解调。纯 numpy，板上可直接跑。

与本地仿真验证过的版本（sim_verify2.py）一致，仅增加：
  * acquire() 只做粗定位（FFT 相关），把逐样点扫描限制在局部窗口内，便于长缓冲；
  * 帧结构带零保护段，避免相邻帧的 RRC 拖尾互相污染前导。
"""
import numpy as np

SPS = 8            # 每符号样点数
ROLLOFF = 0.35     # RRC 滚降
SPAN = 6           # RRC 单侧符号数
N_PRE = 64         # ZC 前导长度（符号）
ZC_ROOT = 25       # ZC 根序号
GUARD_SYM = 16     # 帧间零保护（符号）


# ---------------------------------------------------------------- 基础构件
def rrc_filter(sps, span=SPAN, rolloff=ROLLOFF):
    N = span * sps
    t = np.arange(-N, N + 1) / sps
    h = np.zeros_like(t)
    for i, ti in enumerate(t):
        if abs(ti) < 1e-12:
            h[i] = 1.0 - rolloff + 4.0 * rolloff / np.pi
        elif abs(abs(4 * rolloff * ti) - 1.0) < 1e-9:
            h[i] = (rolloff / np.sqrt(2)) * (
                (1 + 2 / np.pi) * np.sin(np.pi / (4 * rolloff))
                + (1 - 2 / np.pi) * np.cos(np.pi / (4 * rolloff)))
        else:
            h[i] = (np.sin(np.pi * ti * (1 - rolloff))
                    + 4 * rolloff * ti * np.cos(np.pi * ti * (1 + rolloff))) / (
                np.pi * ti * (1 - (4 * rolloff * ti) ** 2))
    return h / np.sqrt(np.sum(h ** 2))


def pulse_shape(symbols, h, sps):
    up = np.zeros(len(symbols) * sps, dtype=complex)
    up[::sps] = symbols
    return np.convolve(up, h, mode="full")


def zc(n=N_PRE, root=ZC_ROOT):
    m = np.arange(n)
    return np.exp(-1j * np.pi * root * m * (m + 1) / n)


def mod_qpsk(bits):
    a = np.asarray(bits)
    if a.size % 2:
        raise ValueError("比特数必须为偶数")
    if a.size and (a.min() < 0 or a.max() > 1):
        raise ValueError("传入的不是比特数组（uint8 下 1-2*b 会溢出成 255）")
    b = a.reshape(-1, 2).astype(np.float64)
    return ((1 - 2 * b[:, 0]) + 1j * (1 - 2 * b[:, 1])) / np.sqrt(2)


def demod_qpsk(sym):
    return np.stack([(sym.real < 0), (sym.imag < 0)],
                    axis=1).astype(np.uint8).reshape(-1)


def interp_cubic(x, t):
    n = np.floor(t).astype(int)
    mu = t - n
    i0 = np.clip(n - 1, 0, len(x) - 1)
    i1 = np.clip(n, 0, len(x) - 1)
    i2 = np.clip(n + 1, 0, len(x) - 1)
    i3 = np.clip(n + 2, 0, len(x) - 1)
    x0, x1, x2, x3 = x[i0], x[i1], x[i2], x[i3]
    a0 = -0.5 * x0 + 1.5 * x1 - 1.5 * x2 + 0.5 * x3
    a1 = x0 - 2.5 * x1 + 2.0 * x2 - 0.5 * x3
    a2 = -0.5 * x0 + 0.5 * x2
    a3 = x1
    return ((a0 * mu + a1) * mu + a2) * mu + a3


def xcorr_valid(a, b):
    n = len(a) + len(b) - 1
    nf = 1 << (n - 1).bit_length()
    c = np.fft.ifft(np.fft.fft(a, nf) * np.conj(np.fft.fft(b, nf)))
    return c[: len(a) - len(b) + 1]


def pll_track(y, f0=0.0, ph0=0.0, kp=0.05, ki=1e-4):
    n = len(y)
    out = np.empty(n, dtype=complex)
    ph = ph0 - f0
    f = f0
    for i in range(n):
        ph += f
        z = y[i] * np.exp(-1j * ph)
        d = complex(1.0 if z.real >= 0 else -1.0,
                    1.0 if z.imag >= 0 else -1.0) / np.sqrt(2)
        e = np.angle(z * np.conj(d))
        ph += kp * e
        f += ki * e
        out[i] = z * np.exp(-1j * kp * e)
    return out


# ---------------------------------------------------------------- 发送侧组帧
def frame_symbols(bits, n_pre=N_PRE):
    """[ZC 前导][数据][零保护]，零保护已含 RRC 拖尾余量。"""
    return np.concatenate([zc(n_pre, ZC_ROOT), mod_qpsk(bits),
                           np.zeros(GUARD_SYM, dtype=complex)])


def build_burst(payloads, h, sps=SPS, lead=None, tail=None, peak=0.45):
    """把多帧拼成一个突发。返回 (complex 基带, 前导样点数, 每帧样点数)。

    归一化按 **RMS**（而不是峰值）：QPSK 的 PAPR 约 6 dB，不同数据图样的包络差别很大
    （全 0/全 1 是恒包络，随机数据起伏大）。按峰值归一会让随机帧电平被压到 1/10，
    导致 SNR 不均、突发定位阈值漏检。这里先定 RMS，再检查峰值不越界。
    """
    spf = len(payloads[0]) // 2 + N_PRE + GUARD_SYM      # 每帧符号数
    n_sym = spf * len(payloads)
    if lead is None:
        lead = 64 * sps
    if tail is None:
        tail = 64 * sps
    body = np.zeros(n_sym * sps, dtype=complex)
    for k, b in enumerate(payloads):
        s = frame_symbols(b)
        assert len(s) == spf, "所有帧的数据长度必须一致"
        body[k * spf * sps:(k + 1) * spf * sps] = pulse_shape(s, h, sps)[
            : spf * sps]

    # 只按有效段（除保护零）算 RMS
    act = np.abs(body) > 1e-12
    rms = float(np.sqrt(np.mean(np.abs(body[act]) ** 2))) if act.any() else 0.0
    if rms > 0:
        body = body / rms * (peak / 1.9)          # 随机 QPSK 峰值≈1.9×RMS
    pk = float(np.max(np.abs(body))) if len(body) else 0.0
    if pk > peak:                                  # 恒包络帧不会触发，兜底防削顶
        body = body * (peak / pk)

    bb = np.concatenate([np.zeros(lead, dtype=complex), body,
                         np.zeros(tail, dtype=complex)])
    return bb, lead, spf * sps


def to_words(bb):
    """complex 基带 → RFSoC 字（低 16 位 I，高 16 位 Q，均为 int16）。"""
    i = np.clip(np.round(bb.real * 32767.0), -32768, 32767).astype(np.int64)
    q = np.clip(np.round(bb.imag * 32767.0), -32768, 32767).astype(np.int64)
    return (((q & 0xFFFF) << 16) | (i & 0xFFFF)).astype(np.uint32)


def from_words(w):
    w = np.asarray(w, dtype=np.uint32)
    i = (w & 0xFFFF).astype(np.uint16).view(np.int16).astype(np.float64)
    q = (w >> 16).astype(np.uint16).view(np.int16).astype(np.float64)
    return i + 1j * q


# ---------------------------------------------------------------- 接收侧
def find_active(x, block=1024, frac=0.05, pad=1):
    """按块能量定位突发区间，返回 (起, 止) 样点下标。

    基线取 5% 分位数（代表"只有直流/静默"的块）：突发可能占掉缓冲的大半，此时
    中位数本身就是信号电平，会把阈值抬到只剩最强块，起始块被漏掉。

    阈值必须足够低：突发首块只被信号覆盖一部分（如 254/1024 点），其能量只有峰值的
    25% 左右。早期用 frac=0.25 时这类"半满块"恰好被漏掉，窗口起点落到信号内部 ——
    由此去直流探针取到前导的局部均值、第 0 帧前导被切掉、整体错位一帧（BER 50~70%）。

    另加 pad 个块的前后余量，保证窗口一定包住突发首尾（DAC 插值滤波器会让上升沿
    拖 10 个样点左右）。
    """
    n = len(x) // block * block
    if n < 4 * block:
        return 0, len(x)
    e = np.abs(x[:n]).astype(np.float64) ** 2
    e = e.reshape(-1, block).mean(axis=1)
    base = float(np.percentile(e, 5))
    span = float(e.max()) - base
    if span <= 0:
        return 0, len(x)
    m = (e - base) > frac * span
    idx = np.flatnonzero(m)
    if len(idx) == 0:
        return 0, len(x)
    a = max(0, int(idx[0]) - pad) * block
    b = min((int(idx[-1]) + 1 + pad) * block, len(x))
    return a, b


def estimate_dc(x, block=1024, factor=4.0):
    """用整包的"静默块"估计 ADC/DAC 直流失调。

    不能只探窗口开头几十点：窗口起点一旦落进信号里，探到的就是数据均值（ZC 前导
    某些段的局部均值可达 -1500+1100j），减掉它等于给星座加一个固定偏移。

    判据用"相对最低能量块"而不是"最低的 20% 块"：后者在缓冲较短、突发占了大部分
    时会把信号块选进来。板上整包 4.19 M 字、突发只占 1~2%，但本地小缓冲测试里突发
    可能占 97%，此时 20% 分位数落在信号能量范围内，估出的假直流可达 5000 LSB，
    足以把前导相位斜率带偏成 -13 kHz。
    """
    n = len(x) // block * block
    if n < 2 * block:
        return complex(float(np.median(x.real)), float(np.median(x.imag)))
    e = np.abs(x[:n]).astype(np.float64) ** 2
    e = e.reshape(-1, block).mean(axis=1)
    e_min = float(e.min())
    sel = np.flatnonzero(e <= e_min * factor + 1.0)
    if len(sel) == 0:
        sel = np.array([int(np.argmin(e))])
    v = x[:n].reshape(-1, block)[sel].ravel()
    return complex(float(np.median(v.real)), float(np.median(v.imag)))


def burst_crest(x, bb_len, block=1024):
    """突发窗口内的波峰因数，用来判定这次环回抓到的信号是否可用。

    干净的单边带 QPSK（RRC 成型、恒模）波峰因数约 1.6。
    若上下两个边带同时进来（等效收到实信号 s+conj(s)），包络会周期性过零，
    波峰因数升到 3.4 以上，且 |corr(mf,p)| 与 |corr(mf,conj(p))| 完全相等。
    这种状态下解调必然崩（BER 30%~50%），必须重载 overlay 重新抓。

    板上实测：每次下载 bitstream 会随机落到这两种状态之一（约 1/4 概率干净），
    同一次加载内保持不变，所以只能在启动时自检、不合格就重载。
    """
    a, _ = find_active(x)
    seg = x[a:a + bb_len + 2 * block]
    if len(seg) < 4 * block:
        return 0.0
    amp = np.abs(seg - estimate_dc(x))
    rms = float(np.sqrt(np.mean(amp ** 2)))
    return float(amp.max() / rms) if rms > 0 else 0.0


def acquire(mf, p, sps, n_pre, fs, cfo_max=60e3):
    """粗捕获：自适应步长扫频，返回 (相关峰滞后, 粗频偏)。

    峰滞后 k_peak 与前导起始 t0 的关系为 t0 = k_peak + h_delay（见 frame_sync）。
    """
    n_idx = np.arange(len(mf))
    T_pre = n_pre * sps / fs
    step = float(np.clip(0.25 / T_pre, 50.0, 2.0e5))
    if cfo_max > 0:
        fl = np.arange(0.0, cfo_max + step / 2, step)
        fl = np.concatenate([-fl[::-1], fl[1:]])
    else:
        fl = np.array([0.0])
    pk = []
    for f in fl:
        y = mf if f == 0.0 else mf * np.exp(-1j * 2 * np.pi * f * n_idx / fs)
        pk.append(np.abs(xcorr_valid(y, p)).max())
    f_c = float(fl[int(np.argmax(pk))])
    y = mf if f_c == 0.0 else mf * np.exp(-1j * 2 * np.pi * f_c * n_idx / fs)
    a = np.abs(xcorr_valid(y, p))
    # 突发里每帧前导等幅，argmax 会随机锁到中间某一帧。取"最早"的强峰，
    # 这样后面按 center + i*帧长 递推才是从第 0 帧开始的。
    loc = np.flatnonzero((a[1:-1] >= a[:-2]) & (a[1:-1] >= a[2:])) + 1
    thr = 0.75 * float(a.max())
    cand = loc[a[loc] >= thr] if len(loc) else np.array([], int)
    k_peak = int(cand[0]) if len(cand) else int(np.argmax(a))
    return k_peak, f_c, step, float(np.max(a)) / (n_pre ** 0.5)


def frame_sync(mf, h, pre, sps, n_pre, n_data, fs,
               center, span=None, cfo_max=60e3, nfft=1024):
    """在 center 附近精捕获并解调一帧。返回 (bits, 符号, info)。"""
    RS = fs / sps
    h_delay = (len(h) - 1) // 2
    need = (n_pre + n_data) * sps + 4
    m_idx = np.arange(n_pre)
    span = span if span is not None else 3 * sps

    lo = max(1, center - span)
    hi = min(len(mf) - need - 2, center + span)
    if hi <= lo:
        raise ValueError("搜索窗口为空：center=%d, 缓冲 %d, 需要 %d"
                         % (center, len(mf), need))
    cands = np.arange(lo, hi + 1)

    # 阶段0：局部粗频偏（步长自适应，取窗内相关峰最大的频点）
    T_pre = n_pre * sps / fs
    step = float(np.clip(0.25 / T_pre, 50.0, 2.0e5))
    fl = np.arange(0.0, cfo_max + step / 2, step)
    fl = np.concatenate([-fl[::-1], fl[1:]])
    p = pulse_shape(pre, h, sps)
    seg = mf[max(0, lo - 4 * sps):min(len(mf), hi + need + 4 * sps)]
    if len(seg) < len(p) + 8:
        raise ValueError("局部片段太短")
    best_f, best_pk = 0.0, -1.0
    n_seg = np.arange(len(seg))
    for f in fl:
        y = seg if f == 0.0 else seg * np.exp(-1j * 2 * np.pi * f * n_seg / fs)
        pk = float(np.abs(xcorr_valid(y, p)).max())
        if pk > best_pk:
            best_f, best_pk = float(f), pk
    f_coarse = best_f
    mfc = mf * np.exp(-1j * 2 * np.pi * f_coarse * np.arange(len(mf)) / fs)

    # 阶段1：逐样点 + 带限 FFT 相干增益（对频偏盲）
    # 带宽必须受限：ZC 相位为二次型，平移 1 符号 ≡ -ZC_ROOT/N_PRE*Rs 的频偏
    w_lim = 2 * np.pi * (step * 0.5 + 200.0) / RS
    j_lo = max(int(nfft // 2 - w_lim / (2 * np.pi) * nfft), 0)
    j_hi = min(int(nfft // 2 + w_lim / (2 * np.pi) * nfft) + 1, nfft)
    Y = mfc[cands[:, None] + m_idx[None, :] * sps] * np.conj(pre)[None, :]
    Z = np.abs(np.fft.fftshift(np.fft.fft(Y, nfft, axis=1), axes=1))[:, j_lo:j_hi]
    jj = np.argmax(Z, axis=1)
    gains = Z[np.arange(len(cands)), jj] / n_pre
    b = int(np.argmax(gains))
    t_c = int(cands[b])
    w_res = 2 * np.pi * (j_lo + jj[b] - nfft // 2) / nfft

    # 阶段2：分数定时
    der = np.exp(-1j * w_res * m_idx)
    best2 = (-1.0, None)
    for df in np.arange(-1.0, 1.0, 0.02):
        t = t_c + df
        if t < 0 or t + need >= len(mfc):
            continue
        y = interp_cubic(mfc, t + m_idx * sps)
        g = abs(np.mean(y * np.conj(pre) * der))
        if g > best2[0]:
            best2 = (g, t)
    if best2[1] is None:
        raise ValueError("分数定时搜索全部越界")
    t0 = best2[1]

    # 阶段3：前导复增益 / 初相 / 残余频偏
    y_pre = interp_cubic(mf, t0 + m_idx * sps)
    y_dat = interp_cubic(mf, t0 + (n_pre + np.arange(n_data)) * sps)
    gc = y_pre * np.conj(pre)
    W, P = np.polyfit(m_idx, np.unwrap(np.angle(gc)), 1)
    amp = float(np.mean(np.abs(gc)))
    if amp <= 0:
        raise ValueError("前导相干增益为 0")

    # 阶段4：二阶判决引导锁相环
    s_est = pll_track(y_dat / amp, f0=W, ph0=P + W * n_pre)
    info = dict(t0=t0, amp=amp, ph0=P, slope=W, gain=best2[0],
                cfo_coarse=f_coarse, cfo_hz=W / (2 * np.pi) * RS,
                cand_gain_c=float(gains[b]))
    return demod_qpsk(s_est), s_est, info


def evm_percent(sym, ref):
    return float(np.sqrt(np.mean(np.abs(sym - ref) ** 2)
                         / np.mean(np.abs(ref) ** 2)) * 100.0)
