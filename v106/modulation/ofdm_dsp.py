# -*- coding: utf-8 -*-
"""OFDM 基带 DSP：组帧、同步、信道估计、解调、指标。

与 mod_dsp 的五个单载波案例共用同一条硬件口径（v106 位流、256 MHz 字率、
NCO 1000 MHz、DMA 单次突发环回），区别只在"怎么把比特铺到时频网格上"。

参数（全部由 256 MHz 字率直接决定）
------------------------------------------------------------------
  FFT N = 1024           子载波间隔 250 kHz，符号 4 µs
  CP    = 128            保护间隔 0.5 µs（12.5%），吸收定时误差与环回拖尾
  占用  |k| ∈ [8, 60]     ±15 MHz —— 与 QPSK 那套 32 MBaud 的带宽同量级，
                          离 256 MHz 字率的奈奎斯特边缘很远，模拟通道不滚降
  导频  |k| % 8 == 0      ±8…±56 共 14 个，BPSK 已知符号
  数据  其余 92 个        QPSK，Gray 映射（复用 mod_dsp.Const）
------------------------------------------------------------------
帧结构（每帧 9728 点 = 38 µs）
  [训练符号 1: CP+T][训练符号 2: CP+T][6 个数据符号: CP+T][512 点零保护]
  T = IFFT(训练子载波)，两个相同的训练符号同时给出定时、频偏与逐子载波信道。

为什么用两个相同的训练符号而不是 ZC 前导
  1. 训练符号天然带限（能量只在 ±15 MHz），ZC 直接在 256 MHz 上发是 ±128 MHz
     宽带，模拟通道边缘会把它削出旁瓣；
  2. 两个相同符号之间的相位差可以直接量 CFO（Schmidl-Cox 的做法），一个符号
     只能量到 |Δf| 的混叠；
  3. 训练符号的频域值已知，加窗 FFT 后就是逐子载波的 LS 信道估计，数据符号
     用同一份 H，不需要插值。

环回通道
  DAC 上混频与 ADC 下混频符号约定相反，接收基带是发送基带的共轭。本模块先用
  |corr(r,T)| 与 |corr(r,conj(T))| 自判方向，共轭时整段取 conj 再处理 ——
  对 OFDM 这一步是必须的：共轭等于频谱左右翻转，逐子载波信道估计救不回来。

本模块只依赖 qpsk_dsp 的位/字转换、突发定位与 FFT 相关，不 import pynq。
"""
import os
import sys

import numpy as np

for _p in (os.environ.get("QPSK_DIR"), "/home/xilinx/qpsk",
           "/home/xilinx/jupyter_notebooks/qpsk"):
    if _p and os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import qpsk_dsp as Q

# ------------------------------------------------------------------ 参数
FS_DEFAULT = 256e6

N = 1024                 # FFT 点数
CP = 128                 # 循环前缀
N_SYM = 6                # 每帧数据符号数
GUARD = 512              # 帧尾零保护
LEAD = 2048              # 突发前的静默
TRAIN_REP = 2            # 训练符号重复次数
SPACING = FS_DEFAULT / N  # 250 kHz

K_MIN, K_MAX, K_STEP = 8, 60, 8   # 占用 |k| ∈ [8, 60]；导频 |k| 为 8 的倍数
K_USED = np.array([k for k in range(-K_MAX, K_MAX + 1)
                   if K_MIN <= abs(k) <= K_MAX])
K_PILOT = np.array([k for k in K_USED if abs(k) % K_STEP == 0])
K_DATA = np.array([k for k in K_USED if abs(k) % K_STEP != 0])

N_BITS = 1024            # 每帧载荷比特（不足的网格位置补 0，不计入 BER）
SYM_LEN = N + CP
TRAIN_LEN = TRAIN_REP * SYM_LEN
FRAME_LEN = TRAIN_LEN + N_SYM * SYM_LEN + GUARD
CAP_BITS = len(K_DATA) * N_SYM * 2      # 每帧网格容量（QPSK）

# 训练符号与导频的已知值：固定种子，收发两侧一致
_RNG = np.random.default_rng(2024)
XT = np.zeros(N, complex)
XT[K_USED % N] = 1.0 - 2.0 * _RNG.integers(0, 2, len(K_USED))
PILOT = XT[K_PILOT % N].copy()
T_TRAIN = np.fft.ifft(XT)               # 单位功率归一化前的时域训练符号


# ============================================================ 发送侧
def _pad_bits(bits, n=CAP_BITS):
    b = np.asarray(bits, np.uint8).reshape(-1)
    if len(b) > n:
        raise ValueError("每帧最多 %d bit，收到 %d" % (n, len(b)))
    if len(b) < n:
        b = np.concatenate([b, np.zeros(n - len(b), np.uint8)])
    return b


def map_frame(bits, const):
    """一帧比特 → 6 个时域 OFDM 符号（含 CP）的列表。"""
    b = _pad_bits(bits)
    syms = const.map(b)
    out = []
    for s in range(N_SYM):
        X = np.zeros(N, complex)
        X[K_PILOT % N] = PILOT
        X[K_DATA % N] = syms[s * len(K_DATA):(s + 1) * len(K_DATA)]
        x = np.fft.ifft(X)
        out.append(np.concatenate([x[-CP:], x]))
    return out


def _train_sym():
    x = T_TRAIN
    return np.concatenate([x[-CP:], x])


def build_burst(bits_list, const, peak=0.4, lead=LEAD, tail=GUARD):
    """多帧比特 → 单个突发（复数基带，已按 peak 归一化）。"""
    parts = [np.zeros(lead, complex)]
    for bits in bits_list:
        parts.append(_train_sym())
        parts.append(_train_sym())
        parts.extend(map_frame(bits, const))
        parts.append(np.zeros(GUARD, complex))
    parts.append(np.zeros(tail, complex))
    bb = np.concatenate(parts)
    mx = float(np.max(np.abs(bb)))
    if mx <= 0:
        raise ValueError("空突发")
    bb = bb * (float(peak) / mx)
    return bb


def papr_db(bb):
    p = np.abs(bb) ** 2
    m = p.max()
    if m <= 0:
        return 0.0
    return float(10 * np.log10(m / p.mean()))


# ============================================================ 接收侧
def _local_peaks(ac, thr, min_gap=64):
    idx = np.flatnonzero((ac[1:-1] > ac[:-2]) & (ac[1:-1] >= ac[2:])) + 1
    idx = idx[ac[idx] > thr]
    if len(idx) == 0:
        return idx
    keep = [int(idx[0])]
    for k in idx[1:]:
        if k - keep[-1] >= min_gap:
            keep.append(int(k))
    return np.array(keep, np.int64)


def synchronize(x, fs=FS_DEFAULT, n_frames=None, verbose=False):
    """定位每一帧并估 CFO / 共轭方向。

    返回 dict(p0, cfo, conjugated, peaks, frame_starts)。
    p0 是第一帧第一个训练符号 T 的起点（已跳过它的 CP）。
    """
    a, b = Q.find_active(x)
    seg = x[max(0, a - 256):min(len(x), b + 256)]
    off = max(0, a - 256)

    c_n = np.abs(Q.xcorr_valid(seg, T_TRAIN))
    c_c = np.abs(Q.xcorr_valid(seg, np.conj(T_TRAIN)))
    conjugated = float(c_c.max()) > float(c_n.max())
    r = np.conj(seg) if conjugated else seg

    ac = np.abs(Q.xcorr_valid(r, T_TRAIN))
    pk = _local_peaks(ac, 0.5 * float(ac.max()))
    if len(pk) == 0:
        raise RuntimeError("训练符号相关峰没找到（波峰因数高？环回断？）")
    ps = int(pk[0])                            # 首峰在 seg 里的下标
    p0 = ps + off                              # 第一帧 T 的起点（整包下标）

    # CFO：两个训练符号之间的相位差（共轭已校正，只剩残余）
    #   r[n+lag] = r[n]·exp(j2πΔf·lag/fs)  →  Δf = -angle(Σ r[n]·conj(r[n+lag]))·fs/2πlag
    lag = SYM_LEN
    w = min(N, FRAME_LEN - lag, len(r) - ps - lag)
    if w > 64:
        z = np.sum(r[ps:ps + w] * np.conj(r[ps + lag:ps + lag + w]))
        cfo = float(-np.angle(z) * fs / (2 * np.pi * lag))
    else:
        cfo = 0.0

    starts = [p0 + f * FRAME_LEN for f in range(n_frames or 0)]
    if verbose:
        gaps = np.diff(pk)
        print("  活跃区 [%d, %d)，相关峰 %d 个，首峰 %d，峰间距 %s"
              % (a, b, len(pk), p0, gaps[:6] if len(gaps) else "—"))
        print("  共轭方向 %s，CFO %.1f Hz" % ("是" if conjugated else "否", cfo))
    return dict(p0=p0, cfo=cfo, conjugated=conjugated, peaks=pk + off,
                starts=starts)


def _fft_win(r, start):
    """取 N 点做 FFT。**不做 fftshift**：子载波 k 直接落在 k % N 上，
    组帧时 X[K_USED % N] 用的是同一套下标，两端口径必须一致。"""
    return np.fft.fft(r[start:start + N])


def channel_estimate(r, p0):
    """两个训练符号 → 逐子载波 LS 信道 H（长度 N 的复数数组，未用子载波为 0）。"""
    H = np.zeros(N, complex)
    acc = np.zeros(len(K_USED), complex)
    for j in range(TRAIN_REP):
        R = _fft_win(r, p0 + j * SYM_LEN)
        acc += R[K_USED % N] / XT[K_USED % N]
    H[K_USED % N] = acc / TRAIN_REP
    return H


def demod(x, fs, frames_bits, const, cfo=None, verbose=False):
    """整包接收缓冲 → 每帧的比特/指标。

    frames_bits : 每帧真值比特（长度 = N_BITS，不含补零）
    返回 list[dict(tag, bits, ideal, err, evm, margin)] 和同步信息。
    """
    a, b = Q.find_active(x)
    xc = x - Q.estimate_dc(x)
    sync = synchronize(xc, fs, len(frames_bits), verbose=verbose)
    r = np.conj(xc) if sync["conjugated"] else xc
    p0 = sync["p0"]

    # CFO 校正（CFO 极小，但校正后残余相移可以忽略，图上也干净）
    f_cfo = sync["cfo"] if cfo is None else float(cfo)
    if f_cfo:
        n = np.arange(len(r))
        r = r * np.exp(-1j * 2 * np.pi * f_cfo * n / fs)

    out = []
    for i, bits in enumerate(frames_bits):
        fs0 = p0 + i * FRAME_LEN
        H = channel_estimate(r, fs0)
        ideal = const.map(_pad_bits(bits))
        got = []
        evms = []
        margins = []
        cpes = []
        for s in range(N_SYM):
            # fs0 是训练符号 T 的起点（已跳过它的 CP）。从"训练 1 的 CP 起点"
            # 量起，两个训练符号占 TRAIN_LEN，所以第 s 个数据符号的 T 起点是
            # fs0 + TRAIN_LEN + s·SYM_LEN —— 不能再加 CP，否则窗滑进下一个 CP。
            st = fs0 + TRAIN_LEN + s * SYM_LEN
            R = _fft_win(r, st)
            # 公共相位：导频处 R/(H·P) 的相位均值（残余 CFO / 相位噪声）
            z = R[K_PILOT % N] / (H[K_PILOT % N] * PILOT)
            cpe = np.angle(np.mean(z))
            cpes.append(float(cpe))
            Y = R[K_DATA % N] / H[K_DATA % N] * np.exp(-1j * cpe)
            got.append(Y)
            ref = ideal[s * len(K_DATA):(s + 1) * len(K_DATA)]
            evms.append(float(np.sqrt(np.mean(np.abs(Y - ref) ** 2)
                                      / np.mean(np.abs(ref) ** 2)) * 100.0))
            margins.append(float(np.min(const.margin(Y))))
        Y = np.concatenate(got)
        bits_rx = const.demap(Y)
        n_truth = len(np.asarray(bits).reshape(-1))
        err = int(np.sum(bits_rx[:n_truth] != np.asarray(bits).reshape(-1)))
        out.append(dict(tag="frame%d" % i, bits=bits_rx, ideal=ideal, Y=Y,
                        H=H, cpes=cpes, err=err, n=n_truth,
                        evm=float(np.max(evms)),
                        margin=float(np.min(margins))))
    return out, sync


def metrics(results, sync, bb_len, fs):
    """把逐帧结果汇成 03 判定用的标量。"""
    err = sum(r["err"] for r in results)
    tot = sum(r["n"] for r in results)
    evm_max = max(r["evm"] for r in results)
    margin_min = min(r["margin"] for r in results)
    return dict(err=err, tot=tot, ber=err / max(tot, 1), evm_max=evm_max,
                margin_min=margin_min, cfo=sync["cfo"],
                conjugated=sync["conjugated"], frames=len(results),
                bb_len=int(bb_len))
