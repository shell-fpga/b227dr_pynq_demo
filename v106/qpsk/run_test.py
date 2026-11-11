"""QPSK 硬件环回测试：用多组不同数据跑 03→04→05 的等价流程。

流程：生成突发 → DMA 单次发射 → 立即整包接收 → 定位突发 → 逐帧精捕获/解调 → BER。

用法（板上 root）：
    python3 run_test.py                 # 默认：3 种帧长 × 多组数据
    python3 run_test.py --quick          # 只跑一轮，快
    python3 run_test.py --bits 1024 --frames 12
"""
import argparse
import sys
import time

import numpy as np

import qpsk_dsp as D
import qpsk_hw as H


# ------------------------------------------------------------------ 数据源
def make_bits(kind, n_bits, seed=0):
    """n_bits 必须是偶数。返回 uint8 比特数组。"""
    n_bits = int(n_bits) // 2 * 2
    rng = np.random.default_rng(seed)
    n = n_bits
    if kind == "rand":
        return rng.integers(0, 2, n, dtype=np.uint8)
    if kind == "zeros":
        return np.zeros(n, np.uint8)
    if kind == "ones":
        return np.ones(n, np.uint8)
    if kind == "alt":                      # 0101...
        return (np.arange(n) & 1).astype(np.uint8)
    if kind == "aa":                       # 0xAA 字节流（展开成比特）
        by = np.frombuffer(b"\xaa" * (n // 8 + 1), np.uint8)
        return np.unpackbits(by, bitorder="big")[:n].copy()
    if kind == "55":                       # 0x55 字节流（展开成比特）
        by = np.frombuffer(b"\x55" * (n // 8 + 1), np.uint8)
        return np.unpackbits(by, bitorder="big")[:n].copy()
    if kind == "cnt":                      # 递增字节 0,1,2,...（展开成比特）
        by = np.tile(np.arange(256, dtype=np.uint8), n // 8 + 1)
        return np.unpackbits(by, bitorder="big")[:n].copy()
    if kind == "prbs9":                    # x^9+x^5+1
        st = 0x1FF
        out = np.empty(n, np.uint8)
        for i in range(n):
            b = ((st >> 8) ^ (st >> 4)) & 1
            out[i] = b
            st = ((st << 1) | b) & 0x1FF
        return out
    if kind == "onehot":                   # 只有第 1 位为 1
        a = np.zeros(n, np.uint8)
        a[1] = 1
        return a
    raise ValueError(kind)


KINDS = ["rand", "zeros", "ones", "alt", "aa", "55", "cnt", "prbs9", "onehot"]


def build_payload_set(n_bits, n_frames, seed0, kinds=None):
    """返回 [(标签, 比特数组), ...]。前若干帧用特殊图样，其余用不同种子的随机数据。"""
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


# ------------------------------------------------------------------ 单次收发
CF_CLEAN = 2.5      # 波峰因数阈值：干净单边带 QPSK ≈1.6，双边带污染 ≥3.4


def open_clean_loopback(probe_words, bb_len, reloads=4, reinits=12,
                        verbose=True, reuse=True):
    """反复初始化，直到抓到干净的环回信号。

    板上实测：每次下载 bitstream 后，ADC 的 I/Q 输出有相当大概率变成"只有 I"——
    虚部恒为 0（Re/Im 能量比 1e16 量级），接收到的等价于实信号，包络周期性过零、
    波峰因数 3.4 以上，解调必然崩到 BER 30%~50%。同一次初始化内状态不会自己变。

    恢复优先级：先只重跑 RFDC 的 deinit/init/NCO（约 0.1 s，实测一次就能恢复），
    仍不行才重下 bitstream（约 2 s）。

    reuse=True（默认）：若 PL 里已经是 v106 位流，第一次不重下，直接承接 01 建好的
    环境；只有自检连续失败才退回"重下位流"这条重路径。

    返回 (loopback, 自检次数, 最后一次的波峰因数)。
    """
    lb = None
    cf = 0.0
    n = 0
    for r in range(reloads):
        # 第一轮允许承接已有 PL 状态；后面几轮是真·重下位流做兜底恢复
        lb = H.Loopback(reuse=(reuse and r == 0), verbose=(verbose and r == 0))
        for k in range(reinits + 1):
            if k > 0:
                lb.rf_reinit()
            G = lb.loop(probe_words)
            x = D.from_words(G["w"])
            x = x[:G["n"]] if G["n"] else x
            cf = D.burst_crest(x, bb_len)
            n += 1
            if verbose:
                print("  环回自检 %2d%s：波峰因数 %.2f  %s"
                      % (n - 1, "" if k else ("（首次建立）" if r == 0
                                              else "（重下 bitstream）"), cf,
                         "OK" if cf < CF_CLEAN else "Q 通道异常，重跑 RFDC 初始化"))
            if cf < CF_CLEAN:
                return lb, n, cf
        del lb
        lb = None
    return lb, n, cf


def run_capture(lb, tags, bits_list, h, sps, peak, verbose=True):
    """一次发射/接收，把每帧都解出来。返回结果列表。"""
    bb, lead, spf_smp = D.build_burst(bits_list, h, sps, peak=peak)
    words = D.to_words(bb)

    t0 = time.time()
    G = lb.loop(words)
    t_cap = time.time() - t0

    x = D.from_words(G["w"])
    n_real = G["n"]
    x = x[:n_real] if n_real else x

    # 突发区间定位（整包 4.19 M 字里只取有信号的一段）。
    # 窗口长度按已知的发送长度取固定值（尾部块能量可能很低，用检测到的终点会截断）；
    # 起点用 find_active 的起点，它已经额外前移一个块做余量。
    a, b = D.find_active(x)
    off = max(0, a)
    seg = x[off:min(len(x), off + len(bb) + 2 * 1024)]

    # 去直流：用整包的静默块估计 ADC/DAC 失调，而不是探窗口开头几十点。
    # 窗口起点可能落在信号内部，那样探到的就是数据均值，减掉等于给星座加固定偏移。
    dc = D.estimate_dc(x)
    seg = seg - dc

    mf = np.convolve(seg, h, mode="full")[: len(seg)]

    if verbose:
        rms = float(np.sqrt(np.mean(np.abs(seg - seg.mean()) ** 2)))
        print("  捕获：收 %d 字(%.2f ms) 突发区 [%d,%d) 段长 %d 墙钟 %.0f ms"
              % (n_real, n_real / lb.fs_word * 1e3, a, b, len(seg), t_cap * 1e3))
        print("        段内 AC rms %.0f LSB，已去直流 %+.0f%+.0fj，峰值 %d LSB"
              % (rms, dc.real, dc.imag, int(np.max(np.abs(seg.real)))))
        print("        TX %s  GAP %.0f us" %
              ("完成" if G["tx_ok"] else "未完成", G["gap_us"]))

    p = D.pulse_shape(D.zc(), h, sps)

    # 频谱方向自判：本板 DAC 上混频与 ADC 下混频用的是同一符号约定，环回输出等于
    # 输入基带的 **共轭**（实测 ±2/4/8/12/16 MHz 单音全部映射成 -f，无频偏无比例误差）。
    # 判据：与 conj(前导) 相关更强就说明收到的是共轭信号，此时把 mf 取共轭即可。
    c_norm = float(np.abs(D.xcorr_valid(mf, p)).max())
    c_conj = float(np.abs(D.xcorr_valid(mf, np.conj(p))).max())
    conjugated = c_conj > c_norm
    if conjugated:
        mf = np.conj(mf)

    k_peak, f_c, step, snr_c = D.acquire(mf, p, sps, D.N_PRE, lb.fs_word,
                                        cfo_max=60e3)
    h_delay = (len(h) - 1) // 2
    center0 = k_peak + h_delay

    if verbose:
        print("        频谱方向：%s（相关 %.3g vs %.3g）"
              % ("共轭（已翻转）" if conjugated else "同相", c_conj, c_norm))

    res = []
    for i, (tag, bits) in enumerate(zip(tags, bits_list)):
        n_data = len(bits) // 2
        center = center0 + i * spf_smp
        try:
            got, sym, inf = D.frame_sync(
                mf, h, D.zc(), sps, D.N_PRE, n_data, lb.fs_word,
                center=center, span=3 * sps, cfo_max=60e3)
            err = int(np.sum(got != bits))
            evm = D.evm_percent(sym, D.mod_qpsk(bits))
            res.append(dict(tag=tag, n=len(bits), ber=err / len(bits), err=err,
                            evm=evm, t0=inf["t0"] + off,
                            cfo=inf["cfo_hz"], ok=(err == 0)))
        except Exception as e:
            res.append(dict(tag=tag, n=len(bits), ber=1.0, err=len(bits),
                            evm=float("nan"), t0=float("nan"),
                            cfo=float("nan"), ok=False, err_msg=str(e)))
    return res, dict(k_peak=k_peak, f_coarse=f_c, acq_snr=snr_c,
                     lead=lead, n_real=n_real, conjugated=conjugated,
                     t_cap_ms=t_cap * 1e3)


def print_table(res, n_bits):
    print("  %-10s %8s %8s %9s %10s %8s" %
          ("数据图样", "比特数", "错误位", "BER", "EVM%", "判决"))
    print("  " + "-" * 60)
    for r in res:
        print("  %-10s %8d %8d %9.2e %10s %8s"
              % (r["tag"], r["n"], r["err"], r["ber"],
                 ("%.2f" % r["evm"]) if r["evm"] == r["evm"] else "  --",
                 "PASS" if r["ok"] else "FAIL"))
    nbad = sum(1 for r in res if not r["ok"])
    tot = sum(r["n"] for r in res)
    err = sum(r["err"] for r in res)
    print("  " + "-" * 60)
    print("  合计 %d 帧 / %d 比特，错 %d 位，BER %.2e，失败帧 %d"
          % (len(res), tot, err, err / max(tot, 1), nbad))
    return nbad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bits", type=int, default=1024)
    ap.add_argument("--frames", type=int, default=9)
    ap.add_argument("--sps", type=int, default=D.SPS)
    ap.add_argument("--peak", type=float, default=0.45)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--seed", type=int, default=100)
    a = ap.parse_args()

    sps = a.sps
    h = D.rrc_filter(sps)
    print("=" * 72)
    print(" QPSK 硬件环回：SPS=%d，Rs = fs_word/%d，RRC α=%.2f span=%d"
          % (sps, sps, D.ROLLOFF, D.SPAN))
    print("=" * 72)

    configs = [(a.bits, a.frames, a.seed)]
    if not a.quick:
        configs = [(256, 9, 200), (1024, 9, 300), (4096, 6, 400)]

    # 启动自检：每次下载 bitstream 可能落到"双边带"坏状态，必须重载到干净为止
    _t, _b = zip(*build_payload_set(configs[0][0], configs[0][1], configs[0][2]))
    probe_bb, _, _ = D.build_burst(list(_b), h, sps, peak=a.peak)
    lb, n_try, cf = open_clean_loopback(D.to_words(probe_bb), len(probe_bb))
    if lb is None:
        print("  ✗ 连续 %d 次重载都没拿到干净环回，检查 SMA 环回线与 01 的 LMK/RFDC 配置"
              % n_try)
        return 2
    print("  环回已就绪：波峰因数 %.2f（自检 %d 次）" % (cf, n_try))

    print("  fs(RF) %.1f MSPS → 字率 %.3f MHz，本符号率 %.3f MBaud"
          % (lb.fs_rf, lb.fs_word / 1e6, lb.fs_word / sps / 1e6))
    st = lb.status()
    for k, v in st.items():
        print("  %s tile：enabled=%s state=0x%X pll=%s"
              % (k, v.get("enabled"), v.get("state", 0), v.get("pll")))
    if st["DAC"].get("state") != 0xF or st["ADC"].get("enabled") != 1:
        print("  ⚠ DAC 未 FULL 或 ADC 未使能 —— 先按 01/02 修上游（LMK/SYSREF）")

    total_bad = 0
    for n_bits, n_frames, seed in configs:
        print("\n" + "-" * 72)
        print(" 帧长 %d 比特（%d 符号 + 前导 %d + 保护 %d）"
              % (n_bits, n_bits // 2, D.N_PRE, D.GUARD_SYM))
        print("-" * 72)
        tags, bitsl = zip(*build_payload_set(n_bits, n_frames, seed))
        res, dbg = run_capture(lb, tags, list(bitsl), h, sps, a.peak)
        cfos = [r["cfo"] for r in res if r["cfo"] == r["cfo"]]
        print("  捕获：峰滞后 %d，频谱方向%s，残余频偏中位数 %+.0f Hz"
              % (dbg["k_peak"],
                 "共轭（已翻转）" if dbg["conjugated"] else "同相",
                 float(np.median(cfos)) if cfos else float("nan")))
        total_bad += print_table(res, n_bits)

    print("\n" + "=" * 72)
    print(" 总结：%s" % ("全部通过（误码为 0）" if total_bad == 0
                        else "%d 帧存在误码" % total_bad))
    print("=" * 72)
    return 0 if total_bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
