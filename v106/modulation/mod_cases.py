# -*- coding: utf-8 -*-
"""五个调制案例的定义、基带生成与接收机流水线。

分工：
  * 本模块只做"数据 ↔ 基带 ↔ 判决"，不 import pynq / 不碰 DMA。
    所以 07/08 两本离线 notebook 断电重启后也能直接跑。
  * 硬件收发在 06 与 run_mod_test.py 里，用 qpsk_hw.Loopback 一次突发收发。

数字案例（ASK/PSK/QAM）共用 QPSK 那套帧结构：
    [64 符号 ZC 前导][n 符号载荷][16 符号零保护]，RRC 成型，9 帧拼一个突发。
模拟案例（AM/FM）连续发送 2.048 ms 的已知单音消息，接收后按已知消息对齐
（验证场景下消息是已知的，这不是盲解调）。
"""
import numpy as np

import mod_dsp as M

SPS = M.SPS
N_PRE = M.N_PRE
GUARD_SYM = M.GUARD_SYM

# ---------------------------------------------------------------- 案例表
DIGITAL_CASES = [
    dict(name="ASK4", scheme="ask", k=2, n_bits=1024, frames=9),
    dict(name="8PSK", scheme="psk", k=3, n_bits=1536, frames=9),
    dict(name="16QAM", scheme="qam", k=2, n_bits=1024, frames=9),
]

ANALOG_CASES = [
    dict(name="AM", kind="am", fc=8e6, fm=0.5e6, mu=0.8, n=524288),
    dict(name="FM", kind="fm", fc=8e6, fm=0.5e6, df=2e6, n=524288),
]

PEAK = 0.4
SEED = 100

# 判决门限（离线仿真：数字 0 误码 / EVM≤3.6% / 余量≥0.91；模拟 corr≥0.999 / rmse≤6%）
LIM = dict(evm_max=8.0, margin_min=0.85,
           corr_min=0.98, rmse_max=0.10, ratio_tol=0.10, ripple_max=0.10)

KINDS = ["rand", "zeros", "ones", "alt", "aa", "55", "cnt", "prbs9", "onehot"]


# ---------------------------------------------------------------- 比特图样
def make_bits(kind, n_bits, seed=0):
    """与 qpsk/run_test.make_bits 同一份实现（那边不能 import，因为它依赖 pynq）。"""
    n_bits = int(n_bits) // 2 * 2
    rng = np.random.default_rng(seed)
    n = n_bits
    if kind == "rand":
        return rng.integers(0, 2, n, dtype=np.uint8)
    if kind == "zeros":
        return np.zeros(n, np.uint8)
    if kind == "ones":
        return np.ones(n, np.uint8)
    if kind == "alt":
        return (np.arange(n) & 1).astype(np.uint8)
    if kind == "aa":
        by = np.frombuffer(b"\xaa" * (n // 8 + 1), np.uint8)
        return np.unpackbits(by, bitorder="big")[:n].copy()
    if kind == "55":
        by = np.frombuffer(b"\x55" * (n // 8 + 1), np.uint8)
        return np.unpackbits(by, bitorder="big")[:n].copy()
    if kind == "cnt":
        by = np.tile(np.arange(256, dtype=np.uint8), n // 8 + 1)
        return np.unpackbits(by, bitorder="big")[:n].copy()
    if kind == "prbs9":
        st = 0x1FF
        out = np.empty(n, np.uint8)
        for i in range(n):
            b = ((st >> 8) ^ (st >> 4)) & 1
            out[i] = b
            st = ((st << 1) | b) & 0x1FF
        return out
    if kind == "onehot":
        a = np.zeros(n, np.uint8)
        a[1] = 1
        return a
    raise ValueError(kind)


def build_payload_set(n_bits, n_frames, seed0=SEED, kinds=None):
    """前若干帧用固定极端图样，其余用不同种子的随机数据。"""
    out = []
    kinds = kinds or KINDS
    for k in kinds:
        if len(out) >= n_frames:
            break
        out.append((k, make_bits(k, n_bits, seed0)))
    s = seed0
    while len(out) < n_frames:
        s += 1
        out.append(("rand#%d" % s, make_bits("rand", n_bits, s)))
    return out[:n_frames]


# ---------------------------------------------------------------- 发送侧
def build_digital(spec, seed=SEED, peak=PEAK, sps=SPS):
    """返回 (const, frames[(tag,bits)], bits_list, bb, lead, spf_smp)。"""
    const = M.Const(spec["scheme"], spec["k"])
    frames = build_payload_set(spec["n_bits"], spec["frames"], seed)
    bits_list = [b for _, b in frames]
    fr_syms = [const.map(b) for b in bits_list]
    h = M.Q.rrc_filter(sps)
    bb, lead, spf_smp = M.build_burst_syms(fr_syms, h, sps, peak=peak)
    return const, frames, fr_syms, bb, lead, spf_smp


def build_analog(spec, fs=256e6):
    """返回 (bb, msg, meta)。"""
    if spec["kind"] == "am":
        return M.am_wave(fs, spec["n"], fc=spec["fc"], fm=spec["fm"],
                         mu=spec["mu"], peak=PEAK)
    return M.fm_wave(fs, spec["n"], fc=spec["fc"], fm=spec["fm"],
                     df=spec["df"], peak=PEAK)


# ---------------------------------------------------------------- 接收侧：数字
def demod_digital(x, fs, const, frames, fr_syms, bb_len, sps=SPS,
                  peak=PEAK, verbose=False):
    """从一整包接收缓冲里解出每帧比特。

    x       : 原始复基带（D.from_words 的输出）
    frames  : [(tag, bits), ...] 真值
    fr_syms : 每帧的发送符号（EVM 参考）
    bb_len  : 发送突发的样点长度（定位用）
    """
    h = M.Q.rrc_filter(sps)
    a, b = M.Q.find_active(x)
    off = max(0, a)
    seg = x[off:min(len(x), off + bb_len + 2048)]
    dc = M.Q.estimate_dc(x)
    seg = seg - dc

    mf_raw = np.convolve(seg, h, mode="full")[:len(seg)]
    p = M.Q.pulse_shape(M.Q.zc(), h, sps)
    c_n = float(np.abs(M.Q.xcorr_valid(mf_raw, p)).max())
    c_c = float(np.abs(M.Q.xcorr_valid(mf_raw, np.conj(p))).max())
    conjugated = c_c > c_n
    mf = np.conj(mf_raw) if conjugated else mf_raw

    k_peak, f_c, step, snr_c = M.Q.acquire(mf, p, sps, N_PRE, fs, cfo_max=60e3)
    h_delay = (len(h) - 1) // 2
    center0 = k_peak + h_delay
    spf_smp = (len(fr_syms[0]) + N_PRE + GUARD_SYM) * sps

    out = []
    for i, ((tag, bits), fr) in enumerate(zip(frames, fr_syms)):
        center = center0 + i * spf_smp
        try:
            syms, info = M.sync_frame(
                mf, h, M.Q.zc(), sps, N_PRE, len(fr), fs,
                center=center, const=const, track=(const.scheme != "ask"))
            got = const.demap(syms)
            err = int(np.sum(got != bits))
            evm = const.evm(syms, bits)
            out.append(dict(tag=tag, n=len(bits), err=err, ber=err / len(bits),
                            evm=evm, margin=float(np.min(const.margin(syms))),
                            t0=info["t0"] + off, cfo=info["cfo_hz"],
                            ok=(err == 0), syms=syms))
        except Exception as e:
            out.append(dict(tag=tag, n=len(bits), err=len(bits), ber=1.0,
                            evm=float("nan"), margin=0.0, t0=float("nan"),
                            cfo=float("nan"), ok=False, err_msg=str(e)))
    dbg = dict(a=a, b=b, off=off, dc=dc, conjugated=conjugated,
               c_n=c_n, c_c=c_c, k_peak=k_peak, f_coarse=f_c,
               acq_snr=snr_c, spf_smp=spf_smp, seg_len=len(seg))
    if verbose:
        print("  活跃区 [%d,%d)  去直流 %+.0f%+.0fj  方向 %s  峰滞后 %d  粗频偏 %+.0f Hz"
              % (a, b, dc.real, dc.imag,
                 "共轭(已翻转)" if conjugated else "同相", k_peak, f_c))
    return out, dbg


# ---------------------------------------------------------------- 接收侧：模拟
def demod_analog(x, fs, spec, msg, bb_len, verbose=False):
    """AM 取包络、FM 鉴频，再按已知消息对齐算指标。"""
    kind = spec["kind"]
    a, b = M.Q.find_active(x)
    dc = M.Q.estimate_dc(x)
    off0 = max(0, a - 65536)
    end = min(len(x), a + bb_len + 65536)
    seg = (x[off0:end] - dc).astype(complex)
    coarse = max(0, a - off0)

    if kind == "am":
        # 包络检测与下变频符号无关：|x·e^{jθ}| = |x|
        rec = np.abs(seg)
        # 单音相关峰每 1/f_m 个样点等高；限制 lag 让对齐窗完整落在活跃区内
        hi_lim = min((b - off0) - len(msg), len(rec) - len(msg))
        lag, corr = M.align_to_msg(rec - rec.mean(), msg, coarse=coarse,
                                   search=4096, win=32768, hi_limit=hi_lim)
        rec_al = rec[lag:lag + len(msg)]
        tm = M.tone_metrics(rec_al, msg, fs, spec["fm"])
        lo, hi = np.percentile(rec_al, [0.5, 99.5])
        mu_meas = float((hi - lo) / (hi + lo))
        extra = dict(mu_meas=mu_meas, mu=spec["mu"])
    else:
        sign, spread = M.fm_pick_sign(seg, fs, spec["fc"], n0=off0)
        y = M.downconvert(seg, fs, spec["fc"], sign, n0=off0)
        inst = M.fm_discrim(y, fs, avg=64)
        hi_lim = min((b - off0) - len(msg), len(inst) - len(msg))
        lag, corr = M.align_to_msg(inst - inst.mean(), msg, coarse=coarse,
                                   search=4096, win=32768, hi_limit=hi_lim)
        rec_al = inst[lag:lag + len(msg)]
        tm = M.tone_metrics(rec_al, msg, fs, spec["fm"])
        dev = float(np.percentile(np.abs(rec_al), 99.5))
        amp = np.abs(y[1 + lag:1 + lag + len(msg)])
        extra = dict(sign=sign, centre_hz=spread, dev=dev, df=spec["df"],
                     ripple=float(np.std(amp) / np.mean(amp)))

    if verbose:
        print("  活跃区 [%d,%d)  去直流 %+.0f%+.0fj  对齐滞后 %d  相关系数 %+.4f"
              % (a, b, dc.real, dc.imag, lag, corr))
    out = dict(kind=kind, lag=lag, corr=corr, thd=tm["thd"], rmse=tm["rmse"],
               gain=tm["gain"], a=a, b=b, dc=dc, rec=rec_al)
    out.update(extra)
    return out


# ---------------------------------------------------------------- 抓包落盘
def lsb_to_words(x):
    """LSB 单位的复基带 → RFSoC 字（与 Q.from_words 互逆，**不再乘 32767**）。

    Q.to_words 的输入是归一化幅度（±1），接收缓冲里的是 ±32767 的整数 LSB，
    直接丢给 to_words 会全部饱和成 ±32768。这里按位重组，数值原样保留。
    """
    x = np.asarray(x)
    i = np.clip(np.round(x.real), -32768, 32767).astype(np.int64)
    q = np.clip(np.round(x.imag), -32768, 32767).astype(np.int64)
    return (((q & 0xFFFF) << 16) | (i & 0xFFFF)).astype(np.uint32)


def save_capture(path, name, x, bb_len, fs_word, extra=None):
    """只存突发前后各 64k 点的一段；整包 4.19 M 字没必要落盘。

    存的是**原始字**（未处理），07 可以据此把接收机每一步重算一遍。
    x 既可传 Q.from_words 出来的复基带（LSB），也可直接传字。
    """
    a, _b = M.Q.find_active(x)
    lo = max(0, int(a) - 65536)
    hi = min(len(x), int(a) + int(bb_len) + 65536)
    w = x[lo:hi]
    if np.iscomplexobj(w):
        w = lsb_to_words(w)
    payload = dict(
        words=np.asarray(w, dtype=np.uint32),
        window_off=np.int64(lo), n_real=np.int64(len(x)),
        fs_word=np.float64(fs_word), bb_len=np.int64(int(bb_len)),
        peak=np.float64(PEAK), name=np.array(name))
    for k, v in (extra or {}).items():
        payload[k] = np.array(v)
    np.savez_compressed(path, **payload)
    return path


# ---------------------------------------------------------------- 离线复算
def load_and_demod(path, verbose=False):
    """从 06 存的 .npz 复算整个接收机，返回统一结构。供 07/08 与命令行共用。"""
    z = np.load(path, allow_pickle=False)
    x = M.Q.from_words(z["words"])
    name = str(z["name"])
    bb_len = int(z["bb_len"])
    fs = float(z["fs_word"])
    ck = str(z["case_kind"])

    if ck == "digital":
        spec = dict(name=name, scheme=str(z["scheme"]), k=int(z["k"]),
                    n_bits=int(z["n_bits"]), frames=int(z["frames"]))
        const, frames, fsyms, bb, lead, spf = build_digital(
            spec, seed=int(z["seed"]))
        res, dbg = demod_digital(x, fs, const, frames, fsyms, bb_len,
                                 verbose=verbose)
        err = sum(r["err"] for r in res)
        tot = sum(r["n"] for r in res)
        cfos = [r["cfo"] for r in res if r["cfo"] == r["cfo"]]
        metrics = dict(err=err, tot=tot, evm_max=max(r["evm"] for r in res),
                       margin_min=min(r["margin"] for r in res),
                       cfo=float(np.median(cfos)) if cfos else float("nan"))
        return dict(name=name, kind="digital", spec=spec, res=res, dbg=dbg,
                    metrics=metrics, x=x)

    kind = str(z["mod"])
    spec = dict(name=name, kind=kind, fc=float(z["fc"]), fm=float(z["fm"]),
                n=int(z["n"]))
    if kind == "am":
        spec["mu"] = float(z["mu"])
    else:
        spec["df"] = float(z["df"])
    bb, msg, meta = build_analog(spec)
    m = demod_analog(x, fs, spec, msg, bb_len, verbose=verbose)
    return dict(name=name, kind="analog", spec=spec, metrics=m, msg=msg, x=x)


def ratio_of(result):
    """模拟案例的"实测/设定"比值：AM 是调制度，FM 是峰值频偏。"""
    m, spec = result["metrics"], result["spec"]
    return (m["mu_meas"] / spec["mu"] if spec["kind"] == "am"
            else m["dev"] / spec["df"])


def verdict(result):
    """返回 (ok, 说明)。门限见 LIM。"""
    m = result["metrics"]
    if result["kind"] == "digital":
        ok = (m["err"] == 0 and m["evm_max"] <= LIM["evm_max"]
              and m["margin_min"] >= LIM["margin_min"])
        det = ("BER %d/%d，EVM max %.2f%%（≤%.1f），余量 min %.3f（≥%.2f）"
               % (m["err"], m["tot"], m["evm_max"], LIM["evm_max"],
                  m["margin_min"], LIM["margin_min"]))
        return ok, det

    ratio = ratio_of(result)
    ok = (abs(m["corr"]) >= LIM["corr_min"]
          and m["rmse"] <= LIM["rmse_max"]
          and abs(ratio - 1) <= LIM["ratio_tol"])
    det = ("corr %+.4f（|·|≥%.2f），rmse %.1f%%（≤%.0f%%），比值 %.3f（1±%.2f）"
           % (m["corr"], LIM["corr_min"], m["rmse"] * 100,
              LIM["rmse_max"] * 100, ratio, LIM["ratio_tol"]))
    if result["spec"]["kind"] == "fm":
        ok = ok and m["ripple"] <= LIM["ripple_max"]
        det += "，纹波 %.1f%%（≤%.0f%%）" % (m["ripple"] * 100,
                                           LIM["ripple_max"] * 100)
    return ok, det
