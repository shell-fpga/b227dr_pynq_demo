# -*- coding: utf-8 -*-
"""五个调制案例的板上环回回归：ASK / 8PSK / 16QAM / AM / FM。

一次初始化，然后逐案例「生成基带 → DMA 单次突发 → 整包抓回 → 解调 → 判决」。
与 notebook 06/07/08 共用 mod_dsp / mod_cases 同一份代码，命令行只是把这些步骤
串起来并打表。

用法（板上 root）：
    python3 run_mod_test.py                 # 五个案例全跑
    python3 run_mod_test.py --case ASK4,FM  # 只跑指定案例
    python3 run_mod_test.py --save /tmp/mod_npz   # 另外存抓包，供 07/08 离线复算
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (HERE, os.path.join(HERE, "..", "qpsk"),
           "/home/xilinx/qpsk", "/home/xilinx/jupyter_notebooks/qpsk"):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import mod_dsp as M       # noqa: E402
import mod_cases as C     # noqa: E402

LIM = C.LIM        # 门限与 08_mod_verify 共用一份


def _cap(lb, words):
    """一次收发，返回 (x, n_real, G)。"""
    G = lb.loop(words)
    x = M.Q.from_words(G["w"])
    n = int(G["n"])
    return (x[:n] if n else x), n, G


def run_digital(lb, spec, save_dir=None, peak=C.PEAK):
    const, frames, fsyms, bb, lead, spf = C.build_digital(spec)
    words = M.Q.to_words(bb)
    t0 = time.time()
    x, n, G = _cap(lb, words)
    t_cap = time.time() - t0
    res, dbg = C.demod_digital(x, lb.fs_word, const, frames, fsyms, len(bb),
                               verbose=True)
    err = sum(r["err"] for r in res)
    tot = sum(r["n"] for r in res)
    evm_max = max(r["evm"] for r in res)
    marg_min = min(r["margin"] for r in res)
    cfos = [r["cfo"] for r in res if r["cfo"] == r["cfo"]]
    ok = (err == 0) and (evm_max <= LIM["evm_max"]) and \
         (marg_min >= LIM["margin_min"])

    print("  捕获 %d 字(%.2f ms) 墙钟 %.0f ms  TX %s  GAP %.0f us"
          % (n, n / lb.fs_word * 1e3, t_cap * 1e3,
             "完成" if G["tx_ok"] else "未完成", G["gap_us"]))
    print("  %-9s %7s %7s %9s %8s %8s" %
          ("图样", "比特", "错位", "BER", "EVM%", "余量"))
    print("  " + "-" * 56)
    for r in res:
        print("  %-9s %7d %7d %9.1e %8.2f %8.3f%s"
              % (r["tag"], r["n"], r["err"], r["ber"], r["evm"], r["margin"],
                 "" if r["ok"] else "   <- " + r.get("err_msg", "")[:40]))
    print("  " + "-" * 56)
    print("  合计 %d 帧 / %d 比特，误 %d 位，BER %.2e；EVM max %.2f%%，"
          "余量 min %.3f；残余频偏中位 %+.0f Hz"
          % (len(res), tot, err, err / max(tot, 1), evm_max, marg_min,
             float(np.median(cfos)) if cfos else float("nan")))

    out = dict(name=spec["name"], kind="digital", err=err, tot=tot,
               evm_max=evm_max, margin_min=marg_min, ok=ok,
               conjugated=dbg["conjugated"], cfo=float(np.median(cfos))
               if cfos else float("nan"), res=res)
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        p = os.path.join(save_dir, "mod_06_%s.npz" % spec["name"])
        C.save_capture(p, spec["name"], x, len(bb), lb.fs_word,
                       extra=dict(case_kind="digital", scheme=spec["scheme"],
                                  k=spec["k"], n_bits=spec["n_bits"],
                                  frames=spec["frames"], seed=C.SEED))
        print("  已存 %s（%.2f MB）" % (p, os.path.getsize(p) / 1e6))
    return out


def run_analog(lb, spec, save_dir=None):
    bb, msg, meta = C.build_analog(spec)
    words = M.Q.to_words(bb)
    t0 = time.time()
    x, n, G = _cap(lb, words)
    t_cap = time.time() - t0
    r = C.demod_analog(x, lb.fs_word, spec, msg, len(bb), verbose=True)

    if spec["kind"] == "am":
        ratio = r["mu_meas"] / spec["mu"]
        ok = (abs(r["corr"]) >= LIM["corr_min"] and
              r["rmse"] <= LIM["rmse_max"] and
              abs(ratio - 1) <= LIM["ratio_tol"])
        detail = ("corr %+.4f  rmse %.1f%%  mu %.3f(设定 %.1f, 比值 %.3f)  THD %.1f%%"
                  % (r["corr"], r["rmse"] * 100, r["mu_meas"], spec["mu"],
                     ratio, r["thd"]))
    else:
        ratio = r["dev"] / spec["df"]
        ok = (abs(r["corr"]) >= LIM["corr_min"] and
              r["rmse"] <= LIM["rmse_max"] and
              abs(ratio - 1) <= LIM["ratio_tol"] and
              r["ripple"] <= LIM["ripple_max"])
        detail = ("corr %+.4f  rmse %.1f%%  频偏 %.3f MHz(设定 %.3f, 比值 %.3f)  "
                  "幅度纹波 %.1f%%  下变频符号 %s"
                  % (r["corr"], r["rmse"] * 100, r["dev"] / 1e6,
                     spec["df"] / 1e6, ratio, r["ripple"] * 100, r["sign"]))

    print("  捕获 %d 字(%.2f ms) 墙钟 %.0f ms  TX %s  GAP %.0f us"
          % (n, n / lb.fs_word * 1e3, t_cap * 1e3,
             "完成" if G["tx_ok"] else "未完成", G["gap_us"]))
    print("  " + detail)

    out = dict(name=spec["name"], kind="analog", ok=ok, corr=r["corr"],
               rmse=r["rmse"], thd=r["thd"], ratio=ratio)
    for k in ("mu_meas", "dev", "ripple", "sign"):
        if k in r:
            out[k] = r[k]
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        p = os.path.join(save_dir, "mod_06_%s.npz" % spec["name"])
        C.save_capture(p, spec["name"], x, len(bb), lb.fs_word,
                       extra=dict(case_kind="analog", mod=spec["kind"],
                                  fc=spec["fc"], fm=spec["fm"], n=spec["n"],
                                  **({"mu": spec["mu"]} if "mu" in spec else {}),
                                  **({"df": spec["df"]} if "df" in spec else {})))
        print("  已存 %s（%.2f MB）" % (p, os.path.getsize(p) / 1e6))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="", help="逗号分隔，如 ASK4,FM；默认全跑")
    ap.add_argument("--save", default="", help="抓包保存目录（供 07/08 离线复算）")
    ap.add_argument("--peak", type=float, default=C.PEAK)
    a = ap.parse_args()

    want = [s.strip().upper() for s in a.case.split(",") if s.strip()]
    specs = [s for s in C.DIGITAL_CASES + C.ANALOG_CASES
             if not want or s["name"].upper() in want]
    if not specs:
        print("没有匹配的案例：", want)
        return 2

    print("=" * 78)
    print(" 调制案例板级环回：ASK / 8PSK / 16QAM / AM / FM")
    print("=" * 78)

    # 时钟：已锁定则 <0.2 s 返回；断电重启后直接跑本脚本也能自举
    import qpsk_init as QI
    try:
        clk = QI.ensure_clock()
    except Exception as e:
        print("  ✗ 时钟初始化失败：%s: %s（需要 root）" % (type(e).__name__, e))
        return 2
    print("  LMK：PLL2 LD=%s  SYSREF MUX=%s  %s"
          % (clk.get("ld2"), clk.get("mux"),
             "（本次写表）" if clk.get("programmed") else "（已就绪）"))

    # 复用 QPSK 的干净环回自检（ADC 偶发只回 I 通道时自动重跑 RFDC）
    import qpsk_dsp as D
    import qpsk_hw as H            # noqa: F401  (确认 pynq 可用)
    import run_test as R
    probe_bits = R.build_payload_set(1024, 9, 300)[:1]
    h_q = D.rrc_filter(D.SPS)
    probe_bb, _, _ = D.build_burst([b for _t, b in probe_bits], h_q, D.SPS,
                                   peak=a.peak)
    lb, n_try, cf = R.open_clean_loopback(D.to_words(probe_bb), len(probe_bb))
    if lb is None:
        print("  ✗ 环回自检失败（%d 次），先查 SMA 环回与 LMK/RFDC" % n_try)
        return 2
    print("  环回就绪：波峰因数 %.2f（自检 %d 次），字率 %.3f MHz"
          % (cf, n_try, lb.fs_word / 1e6))
    st = lb.status()
    print("  DAC0 state=0x%X  ADC0 enabled=%s"
          % (st["DAC"].get("state", -1), st["ADC"].get("enabled")))
    print()

    results = []
    for spec in specs:
        print("-" * 78)
        print(" [%s] %s" % (spec["name"], "数字" if "scheme" in spec else "模拟"))
        print("-" * 78)
        try:
            if "scheme" in spec:
                results.append(run_digital(lb, spec, a.save or None,
                                            peak=a.peak))
            else:
                results.append(run_analog(lb, spec, a.save or None))
        except Exception as e:
            print("  ✗ 异常：%s: %s" % (type(e).__name__, e))
            results.append(dict(name=spec["name"], ok=False, err_msg=str(e)))
        print()

    print("=" * 78)
    print(" 汇总")
    print("=" * 78)
    print("  %-6s %-6s %-6s  %s" % ("案例", "类型", "结果", "关键指标"))
    print("  " + "-" * 74)
    for r in results:
        if r.get("kind") == "digital":
            det = "BER %.1e  EVM %.2f%%  余量 %.3f" % (
                r["err"] / max(r["tot"], 1), r["evm_max"], r["margin_min"])
            kind = "数字"
        elif r.get("kind") == "analog":
            det = "corr %+.4f  rmse %.1f%%  比值 %.3f" % (
                r["corr"], r["rmse"] * 100, r["ratio"])
            kind = "模拟"
        else:
            det = r.get("err_msg", "")
            kind = "?"
        print("  %-6s %-6s %-6s  %s" % (r["name"], kind,
                                        "PASS" if r["ok"] else "FAIL", det))
    nbad = sum(1 for r in results if not r["ok"])
    print("  " + "-" * 74)
    print("  %d 个案例，%s" % (len(results),
                               "全部通过" if nbad == 0 else "%d 个未通过" % nbad))
    return 0 if nbad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
