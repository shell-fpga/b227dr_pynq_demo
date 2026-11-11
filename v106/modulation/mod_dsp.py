# -*- coding: utf-8 -*-
"""AM / FM / ASK / PSK / QAM 通用基带 DSP：成型、组帧、同步、判决、指标。

硬件口径与 qpsk_dsp / qpsk_hw 完全一致（字率 256 MHz、QPSK 那份 128 位 DMA 位流）：

  * 数字调制沿用 QPSK 的帧结构：[64 符号 ZC 前导][数据][16 符号零保护]，RRC 成型；
    ZC 前导只用于定时/频偏/复增益，与载荷的星座无关，所以四种数字调制共用同一套
    捕获代码。
  * 模拟调制（AM/FM）连续发送，不加前导；消息是单音，解调后按"已知消息"对齐。
  * 环回通道把基带取共轭（DAC 上混频与 ADC 下混频符号约定相反）。数字链路先用
    |corr(mf,p)| 与 |corr(mf,conj(p))| 自判方向；AM 取包络本身与混频符号无关；
    FM 用**残余中心频率**自判下变频符号 —— 判错时整个信号留在 ±2fc（16 MHz），
    判对时只剩几百 Hz 的 CFO，两者差四个数量级，不会误判。

本模块只依赖 qpsk_dsp 的基础构件（RRC、ZC、插值、FFT 相关、突发定位），
不复制那些已经板上验证过的算法。qpsk_dsp 在板上位于 /home/xilinx/qpsk/，
所以这里自己把该目录加进 sys.path，import 时不必再由调用方准备。
"""
import os
import sys

import numpy as np

for _p in (os.environ.get("QPSK_DIR"), "/home/xilinx/qpsk",
           "/home/xilinx/jupyter_notebooks/qpsk"):
    if _p and os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import qpsk_dsp as Q

SPS = Q.SPS
ROLLOFF = Q.ROLLOFF
SPAN = Q.SPAN
N_PRE = Q.N_PRE
ZC_ROOT = Q.ZC_ROOT
GUARD_SYM = Q.GUARD_SYM


# ============================================================ 星座映射
def _gray_int(bits2d):
    """把 (n, k) 的比特矩阵变成 Gray 序号 v = b ^ (b>>1)（b 是自然二进制值）。"""
    k = bits2d.shape[1]
    v = np.zeros(bits2d.shape[0], np.int64)
    for j in range(k):
        v = (v << 1) | bits2d[:, j].astype(np.int64)
    return v ^ (v >> 1)


def _int_bits(v, k):
    """Gray 序号 v → (n, k) 比特矩阵（Gray 逆映射 v ^ (v>>1) ...）。"""
    b = np.array(v, np.int64)
    # 逆 Gray：b = g ^ (g>>1) ^ (g>>2) ...
    g = b.copy()
    sh = 1
    while sh < k:
        g ^= (b >> sh)
        sh += 1
    out = np.empty((len(g), k), np.uint8)
    for j in range(k):
        out[:, j] = (g >> (k - 1 - j)) & 1
    return out


def _two_pow(k):
    return 1 << int(k)


class Const(object):
    """M 元星座：map / demap / slice / margin。

    scheme:
      "ask"  —— PAM，M = 2^k 个实电平，归一化到平均功率 1
      "psk"  —— M = 2^k 个单位圆相位，Gray 相邻
      "qam"  —— 方阵 QAM，(2^k)² 个点，I/Q 各自 Gray，整体平均功率 1
    """

    def __init__(self, scheme, k):
        self.scheme = str(scheme).lower()
        self.k = int(k)
        if self.scheme not in ("ask", "psk", "qam"):
            raise ValueError("未知调制 %r" % scheme)
        self.M = (1 << self.k) ** (2 if self.scheme == "qam" else 1)

    # ---------- 发送 ----------
    def map(self, bits):
        b = np.asarray(bits, np.uint8)
        if b.ndim != 1:
            raise ValueError("比特必须是一维")
        k = self.k
        if self.scheme == "ask":
            if b.size % k:
                raise ValueError("比特数必须是 %d 的倍数" % k)
            lev = 2 * _gray_int(b.reshape(-1, k)) - (_two_pow(k) - 1)
            # 归一化必须按完整电平集算，不能按本次数据：全 0 数据只用到最低电平，
            # 按数据算会把归一化系数变成 3 而不是 sqrt(5)，接收端判决器对不上。
            L = _two_pow(k)
            norm = np.sqrt(np.mean((2 * np.arange(L) - (L - 1)) ** 2, dtype=np.float64))
            return lev / norm
        if self.scheme == "psk":
            if b.size % k:
                raise ValueError("比特数必须是 %d 的倍数" % k)
            # 相位序号直接取 Gray 码 s = b ^ (b>>1)：相邻相位（含 0 与 M-1 回绕）
            # 只差一个比特，这就是 8PSK 的 Gray 映射。
            s = _gray_int(b.reshape(-1, k))
            return np.exp(1j * 2 * np.pi * s / (1 << k))
        # QAM
        if b.size % (2 * k):
            raise ValueError("比特数必须是 %d 的倍数" % (2 * k))
        bb = b.reshape(-1, 2 * k)
        i = 2 * _gray_int(bb[:, :k]) - (_two_pow(k) - 1)
        q = 2 * _gray_int(bb[:, k:]) - (_two_pow(k) - 1)
        # 归一化常数必须由**完整星座**算出，不能由本次数据的统计量算出：
        # 全 0 数据只会落在 (-3,-3)，按数据算会得到 sqrt(18) 而不是 sqrt(10)，
        # 星座整体缩放 0.75 倍，判决器与锁相环全部失配。
        L = _two_pow(k)
        scale = np.sqrt(2 * np.mean((2 * np.arange(L) - (L - 1)) ** 2, dtype=np.float64))
        return (i + 1j * q) / scale

    # ---------- 接收 ----------
    def demap(self, sym):
        y = np.asarray(sym)
        k = self.k
        if self.scheme == "ask":
            L = _two_pow(k)
            lev = 2 * np.arange(L) - (L - 1)
            g = np.clip(np.round((y.real * np.sqrt(np.mean(lev ** 2)) + (L - 1)) / 2),
                        0, L - 1).astype(np.int64)
            return _int_bits(g, k).reshape(-1)
        if self.scheme == "psk":
            M = 1 << k
            s = np.mod(np.round(np.angle(y) / (2 * np.pi / M)).astype(np.int64), M)
            return _int_bits(s, k).reshape(-1)
        L = _two_pow(k)
        sc = np.sqrt(2 * np.mean((2 * np.arange(L) - (L - 1)) ** 2))
        gi = np.clip(np.round((y.real * sc + (L - 1)) / 2), 0, L - 1).astype(np.int64)
        gq = np.clip(np.round((y.imag * sc + (L - 1)) / 2), 0, L - 1).astype(np.int64)
        bi = _int_bits(gi, k)
        bq = _int_bits(gq, k)
        return np.concatenate([bi, bq], axis=1).reshape(-1)

    def slice(self, sym):
        """最近星座点（判决引导锁相环用），保持与输入同形状。"""
        y = np.asarray(sym, complex)
        k = self.k
        if self.scheme == "ask":
            L = _two_pow(k)
            lev = (2 * np.arange(L) - (L - 1)) / np.sqrt(np.mean((2 * np.arange(L) - (L - 1)) ** 2))
            d = np.abs(y.real[:, None] - lev[None, :])
            return lev[np.argmin(d, axis=1)].astype(complex)
        if self.scheme == "psk":
            M = 1 << k
            s = np.mod(np.round(np.angle(y) / (2 * np.pi / M)).astype(np.int64), M)
            return np.exp(1j * 2 * np.pi * s / M)
        L = _two_pow(k)
        sc = np.sqrt(2 * np.mean((2 * np.arange(L) - (L - 1)) ** 2))
        vi = (2 * np.arange(L) - (L - 1)) / sc
        di = np.abs(y.real[:, None] - vi[None, :])
        dq = np.abs(y.imag[:, None] - vi[None, :])
        return vi[np.argmin(di, axis=1)] + 1j * vi[np.argmin(dq, axis=1)]

    def margin(self, sym):
        """判决余量 0~1：1 = 星座点中心，0 = 判决边界。

        ASK/QAM 用"到最近的坐标判决边界"除以半个最小间距；PSK 用角距离除以半个
        相位间距。只反映载荷符号本身的可靠性，与噪声/增益无关。
        """
        y = np.asarray(sym)
        k = self.k
        if self.scheme == "ask":
            L = 1 << k
            x = y.real * np.sqrt(np.mean((2 * np.arange(L) - (L - 1)) ** 2))
            bnd = np.array([2 * j - (L - 2) for j in range(L - 1)], np.float64)
            m = np.min(np.abs(x[:, None] - bnd[None, :]), axis=1)
            return np.clip(m, 0, 1)
        if self.scheme == "psk":
            M = 1 << k
            u = np.mod(np.angle(y) / (2 * np.pi / M), 1.0)
            return np.clip(np.abs(u - 0.5) * 2.0, 0, 1)
        L = 1 << k
        sc = np.sqrt(2 * np.mean((2 * np.arange(L) - (L - 1)) ** 2))
        bnd = np.array([2 * j - (L - 2) for j in range(L - 1)], np.float64)
        mi = np.min(np.abs((y.real * sc)[:, None] - bnd[None, :]), axis=1)
        mq = np.min(np.abs((y.imag * sc)[:, None] - bnd[None, :]), axis=1)
        return np.clip(np.minimum(mi, mq), 0, 1)

    def evm(self, sym, bits):
        ref = self.map(bits)
        return float(np.sqrt(np.mean(np.abs(sym - ref) ** 2)
                             / np.mean(np.abs(ref) ** 2)) * 100.0)


# ============================================================ 组帧 / 成型
def build_burst_syms(frames, h, sps=SPS, peak=0.4, lead=None, tail=None):
    """多帧符号序列 → 单个突发（复数基带）。

    frames: [(n_sym,) complex, ...]，所有帧符号数必须相同。
    每帧 = [ZC 前导][载荷][零保护]，与 qpsk_dsp.build_burst 的排布逐样点一致。
    归一化按**峰值**（不是 RMS）：QAM/ASK 的峰均比随图样变化，按峰值归一才能保证
    任何图样都不会削顶；每帧的 EVM 用各自的参考符号，绝对电平不影响判决。
    """
    spf = len(frames[0]) + N_PRE + GUARD_SYM
    for f in frames:
        if len(f) + N_PRE + GUARD_SYM != spf:
            raise ValueError("所有帧的载荷符号数必须相同")
    n_sym = spf * len(frames)
    if lead is None:
        lead = 64 * sps
    if tail is None:
        tail = 64 * sps
    body = np.zeros(n_sym * sps, dtype=complex)
    for k, f in enumerate(frames):
        s = np.concatenate([Q.zc(N_PRE, ZC_ROOT), f,
                            np.zeros(GUARD_SYM, dtype=complex)])
        body[k * spf * sps:(k + 1) * spf * sps] = Q.pulse_shape(s, h, sps)[:spf * sps]
    pk = float(np.max(np.abs(body))) if len(body) else 0.0
    if pk > 0:
        body = body * (peak / pk)
    bb = np.concatenate([np.zeros(lead, dtype=complex), body,
                         np.zeros(tail, dtype=complex)])
    return bb, lead, spf * sps


# ============================================================ 通用逐帧同步
def pll_track(y, decide, f0=0.0, ph0=0.0, kp=0.05, ki=1e-4):
    """二阶判决引导锁相环，判决器由星座提供。"""
    n = len(y)
    out = np.empty(n, dtype=complex)
    ph = ph0 - f0
    f = f0
    for i in range(n):
        ph += f
        z = y[i] * np.exp(-1j * ph)
        d = decide(np.array([z]))[0]
        e = np.angle(z * np.conj(d))
        ph += kp * e
        f += ki * e
        out[i] = z * np.exp(-1j * kp * e)
    return out


def sync_frame(mf, h, pre, sps, n_pre, n_data, fs, center, const,
               track=True, span=None, cfo_max=60e3, nfft=1024):
    """在 center 附近精捕获并解调一帧，返回 (归一化符号, info)。

    与 qpsk_dsp.frame_sync 的阶段 0~3 完全一致，只把最后的"QPSK 判决 + 锁相环"
    换成"由 const 给定的通用判决器"。track=False 时不做锁相环，只用前导拟合的
    相位斜率开环去旋转（ASK 这类实载荷用，避免 0/π 相位模糊把环带翻）。
    """
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

    # 阶段0：局部粗频偏
    T_pre = n_pre * sps / fs
    step = float(np.clip(0.25 / T_pre, 50.0, 2.0e5))
    fl = np.arange(0.0, cfo_max + step / 2, step)
    fl = np.concatenate([-fl[::-1], fl[1:]])
    p = Q.pulse_shape(pre, h, sps)
    seg = mf[max(0, lo - 4 * sps):min(len(mf), hi + need + 4 * sps)]
    if len(seg) < len(p) + 8:
        raise ValueError("局部片段太短")
    best_f, best_pk = 0.0, -1.0
    n_seg = np.arange(len(seg))
    for f in fl:
        y = seg if f == 0.0 else seg * np.exp(-1j * 2 * np.pi * f * n_seg / fs)
        pk = float(np.abs(Q.xcorr_valid(y, p)).max())
        if pk > best_pk:
            best_f, best_pk = float(f), pk
    f_coarse = best_f
    mfc = mf * np.exp(-1j * 2 * np.pi * f_coarse * np.arange(len(mf)) / fs)

    # 阶段1：逐样点 + 带限 FFT 相干增益
    import math
    w_lim = 2 * np.pi * (step * 0.5 + 200.0) / RS
    j_lo = max(int(nfft // 2 - math.ceil(w_lim / (2 * np.pi) * nfft)), 0)
    j_hi = min(int(nfft // 2 + math.ceil(w_lim / (2 * np.pi) * nfft)) + 1, nfft)
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
        y = Q.interp_cubic(mfc, t + m_idx * sps)
        g = abs(np.mean(y * np.conj(pre) * der))
        if g > best2[0]:
            best2 = (g, t)
    if best2[1] is None:
        raise ValueError("分数定时搜索全部越界")
    t0 = best2[1]

    # 阶段3：前导复增益 / 初相 / 残余频偏
    y_pre = Q.interp_cubic(mf, t0 + m_idx * sps)
    y_dat = Q.interp_cubic(mf, t0 + (n_pre + np.arange(n_data)) * sps)
    gc = y_pre * np.conj(pre)
    W, P = np.polyfit(m_idx, np.unwrap(np.angle(gc)), 1)
    amp = float(np.mean(np.abs(gc)))
    if amp <= 0:
        raise ValueError("前导相干增益为 0")

    if track:
        s_est = pll_track(y_dat / amp, const.slice, f0=W, ph0=P + W * n_pre)
    else:
        ph = P + W * (n_pre + np.arange(n_data))
        s_est = y_dat / amp * np.exp(-1j * ph)

    info = dict(t0=t0, amp=amp, ph0=P, slope=W, gain=best2[0],
                cfo_coarse=f_coarse, cfo_hz=W / (2 * np.pi) * RS,
                cand_gain_c=float(gains[b]))
    return s_est, info


# ============================================================ 突发 / 直流
def locate_and_segment(x, bb_len, block=1024, margin=65536):
    """定位突发并截出"前后各留 margin 点"的段（去直流前）。返回 (seg, off, dc)。"""
    a, b = Q.find_active(x)
    off = max(0, int(a) - margin)
    end = min(len(x), int(a) + int(bb_len) + margin)
    return x[off:end].copy(), off, Q.estimate_dc(x)


def cfo_selfcheck(mf, p, sps, fs, cfo_max=60e3):
    """返回 (k_peak, f_coarse, snr_c)；与 run_test 的捕获口径一致。"""
    return Q.acquire(mf, p, sps, N_PRE, fs, cfo_max=cfo_max)


# ============================================================ 模拟调制
def am_wave(fs, n, fc=8e6, fm=0.5e6, mu=0.8, peak=0.4, lead=4096, tail=4096):
    """DSB-LC 调幅：s(t) = [1 + mu*cos(2πfm t)] * exp(j2πfc t)。

    返回 (bb, msg, meta)。msg 是归一化消息（峰值 1），供解调比对。
    """
    t = np.arange(n) / fs
    msg = np.cos(2 * np.pi * fm * t)
    env = 1.0 + mu * msg
    bb = env * np.exp(1j * 2 * np.pi * fc * t)
    bb = bb / np.max(np.abs(bb)) * peak
    msg = msg.astype(np.float64)
    out = np.concatenate([np.zeros(lead, complex), bb, np.zeros(tail, complex)])
    meta = dict(fc=fc, fm=fm, mu=mu, n=n, lead=lead, tail=tail, peak=peak)
    return out, msg, meta


def fm_wave(fs, n, fc=8e6, fm=0.5e6, df=2e6, peak=0.4, lead=4096, tail=4096):
    """FM：s(t) = exp(j(2πfc t + 2π*df*∫m dt))，m(t)=cos(2πfm t)。

    峰值频偏 = df，调制指数 beta = df/fm。
    """
    t = np.arange(n) / fs
    msg = np.cos(2 * np.pi * fm * t)
    phase = 2 * np.pi * fc * t + 2 * np.pi * df * np.cumsum(msg) / fs
    bb = np.exp(1j * phase) * peak
    out = np.concatenate([np.zeros(lead, complex), bb, np.zeros(tail, complex)])
    meta = dict(fc=fc, fm=fm, df=df, beta=df / fm, n=n,
                lead=lead, tail=tail, peak=peak)
    return out, msg.astype(np.float64), meta


def downconvert(x, fs, fc, sign, n0=0):
    """按 sign 下变频：y = x * exp(sign*j*2πfc*(n0+n)/fs)。sign=+1 对应环回共轭。"""
    n = np.arange(len(x)) + n0
    return x * np.exp(sign * 1j * 2 * np.pi * fc * n / fs)


def fm_discrim(y, fs, avg=64, gate=True):
    """正交鉴频 + 低通：d[n] = angle(y[n+1]·conj(y[n]))·fs/2π，再做 avg 点滑动平均。

    必须低通，否则测不准：256 MHz 字率下，单样点相位噪声（30 dB SNR 时约 0.02 rad）
    折成瞬时频率就是 ~0.9 MHz，而频偏只有 ±2 MHz —— 逐样点鉴频等于工作在 FM
    门限区，相关系数只有 0.73。avg=64 时噪声降 8 倍（~0.11 MHz），
    同时滑动平均的零点在 fs/avg = 4 MHz，落在消息带外，不会削掉消息本身。

    还要**幅度门控**：突发前后的静默区只有噪声，相位差是均匀分布的，鉴频出来是
    ±π·fs/2π = ±128 MHz 的满量程跳变（滑动平均后仍有 ~16 MHz），比信号本身
    （±2 MHz）大一个量级。对齐相关时这些静默区的"伪瞬时频率"会把峰值从真实起点
    拖走（实测能偏 500~2000 样点），所以先把低幅度样点清零再做相关。
    返回长度 len(y)-1 的瞬时频率（Hz）。
    """
    amp = np.abs(y)
    d = np.angle(y[1:] * np.conj(y[:-1])) * (fs / (2 * np.pi))
    if gate:
        # 阈值取信号包络的 30%：||y|| 的前 0.5% 分位是信号幅度的稳健估计
        thr = 0.3 * float(np.percentile(amp, 99.5))
        keep = (amp[:-1] > thr) & (amp[1:] > thr)
        d = np.where(keep, d, 0.0)
    avg = int(max(1, avg))
    if avg > 1:
        # 滑动平均的直流增益为 1；mode="same" 保持长度，滞后 (avg-1)/2 由对齐吸收
        d = np.convolve(d, np.ones(avg, np.float64) / avg, mode="same")
    if gate:
        # 平滑会把静默区边缘的噪声抹进来，再按平滑后的局部能量门一次
        thr2 = 0.3 * thr
        e = np.convolve(amp[1:], np.ones(avg, np.float64) / avg, mode="same")
        d = np.where(e > thr2, d, 0.0)
    return d


def fm_pick_sign(x, fs, fc, n0=0, avg=64):
    """用**残余中心频率**判断该用哪个下变频符号。

    正确符号下基带中心在 0（消息单音均值为 0，只剩几百 Hz 的 CFO）；
    错误符号下整个信号留在 ±2fc（这里 16 MHz）。所以直接比较两种符号下
    瞬时频率的中位数即可，比"相位差离散度"稳得多 —— 后者在静默区被噪声
    占满，99% 分位数两种情况都接近 π，根本分不开。
    """
    mag = np.abs(x)
    thr = 0.5 * np.percentile(mag, 95)
    best = None
    for s in (+1, -1):
        y = downconvert(x, fs, fc, s, n0)
        d = fm_discrim(y, fs, avg=avg)
        m = np.abs(y[1:]) > thr
        centre = float(np.median(np.abs(d[m]))) if np.any(m) else float("inf")
        if best is None or centre < best[0]:
            best = (centre, s)
    return best[1], best[0]


def align_to_msg(rec, msg, coarse=0, search=4096, win=32768, hi_limit=None):
    """把 rec 与 msg 对齐：返回 (lag, corr)。

    c[k] = <rec[k:k+W], msg[:W]>，在 [coarse-search, coarse+search] 内取最大。
    corr 在**局部去均值**后算：AM 包络带一个远大于交流分量的直流（载波电平），
    直接用整段均值去减会在窗口里留下残余直流，把相关系数从 0.999 压到 0.94。

    hi_limit：lag 的上界（含）。单音消息每 1/f_m 个样点的相关峰等高，不加约束时
    可能挑到靠后的等高峰，对齐窗尾部越出信号区，最后一段被零值/底噪污染，
    RMSE 从 4% 抬到 13%。调用方传"对齐窗仍完整落在活跃区内"的最大 lag。
    """
    W = int(min(win, len(msg), max(64, len(rec) - 1)))
    lo = max(0, int(coarse) - int(search))
    hi = min(len(rec) - W, int(coarse) + int(search))
    if hi_limit is not None:
        hi = min(hi, int(hi_limit))
    if hi < lo:
        # 约束比窗口起点还靠前：保留约束，把搜索下界收回来，不要退回全窗口搜索
        lo = max(0, hi - int(search))
        if hi < lo:
            lo, hi = 0, max(0, len(rec) - W)
    c = Q.xcorr_valid(rec, msg[:W])
    seg = c[lo:hi + 1]
    if len(seg) == 0:
        return int(coarse), 0.0

    def _corr(k):
        rw = np.asarray(rec[k:k + W], np.float64)
        mw = np.asarray(msg[:W], np.float64)
        rw = rw - rw.mean()
        mw = mw - mw.mean()
        den = np.linalg.norm(rw) * np.linalg.norm(mw)
        return float(np.dot(mw, rw) / den) if den > 0 else 0.0

    # 候选 = 有符号正峰（反相族的相关峰与正相族幅度相同，按 |c| 取会随机落到
    # 负峰，拟合增益为负、波形反相）。纯单音的相关峰每 1/f_m 个样点重复一次，
    # 取**最早**的准等高峰：更早的峰会从信号起点之前开始，相关窗里的零样点
    # 把它的归一化相关系数压低 ~1%，据此可区分。
    sv = np.real(seg)                  # xcorr 可能给复数 dtype，这里只取实部
    peak = np.flatnonzero((sv[1:-1] > sv[:-2]) & (sv[1:-1] >= sv[2:])) + 1
    peak = peak[sv[peak] > 0.5 * float(sv.max())] if len(peak) else peak
    if len(peak) == 0:
        k = lo + int(np.argmax(np.abs(seg)))
        return k, _corr(k)

    ks = lo + peak
    cs = np.array([_corr(int(k)) for k in ks])
    cmax = float(np.max(cs))
    good = np.flatnonzero(cs >= 0.999 * cmax)
    if len(good) == 0:
        good = np.array([int(np.argmax(cs))])
    i = int(good[0])                     # 最早的一个准等高峰
    return int(ks[i]), float(cs[i])


def spectral_metrics(x, fs, f0=0.0, win=8192):
    """加 Hann 窗的幅度谱（dB，归一化到峰值 0 dB）。返回 (freqs, dB)。"""
    x = np.asarray(x)
    K = int(min(win, len(x)))
    seg = x[:K] * np.hanning(K)
    X = np.fft.fftshift(np.fft.fft(seg))
    f = np.fft.fftshift(np.fft.fftfreq(K, 1 / fs)) + f0
    mag = np.abs(X)
    mag = mag / (mag.max() + 1e-30)
    return f, 20 * np.log10(mag + 1e-12)


def tone_metrics(rec, msg, fs, f_tone):
    """对已对齐的单音消息算：相关系数、归一化 RMSE、增益、THD。

    rec 与 msg 等长（或 rec 更长，取前 len(msg)）。THD 在 rec 的前 2^k 个整周期上
    做 FFT：基波在 f_tone，谐波 2f..6f 的 RMS 和 / 基波 RMS。
    """
    n = min(len(rec), len(msg))
    r = np.asarray(rec[:n], np.float64)
    m = np.asarray(msg[:n], np.float64)
    m = m - m.mean()
    r = r - r.mean()
    den = np.linalg.norm(r) * np.linalg.norm(m)
    corr = float(np.dot(r, m) / den) if den > 0 else 0.0
    gain = float(np.dot(r, m) / np.dot(m, m)) if np.dot(m, m) > 0 else 0.0
    rmse = float(np.sqrt(np.mean((r - gain * m) ** 2)) / (abs(gain) * np.sqrt(np.mean(m ** 2)) + 1e-30))
    # THD：取整数个周期，减少泄漏
    per = fs / f_tone
    nper = int(n // per)
    if nper >= 4:
        N = int(round(nper * per))
        w = np.hanning(N)
        X = np.abs(np.fft.rfft((r[:N] - r[:N].mean()) * w))
        bin0 = int(round(f_tone * N / fs))
        if bin0 >= 1:
            fund = float(np.sqrt(np.sum(X[max(0, bin0 - 1):bin0 + 2] ** 2)))
            harm = 0.0
            for h in range(2, 7):
                b = int(round(h * f_tone * N / fs))
                if b >= len(X) - 1:
                    break
                harm += float(np.sum(X[b - 1:b + 2] ** 2))
            thd = float(np.sqrt(harm) / fund) * 100.0 if fund > 0 else float("nan")
        else:
            thd = float("nan")
    else:
        thd = float("nan")
    return dict(corr=corr, gain=gain, rmse=rmse, thd=thd)
