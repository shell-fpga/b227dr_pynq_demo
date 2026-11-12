# -*- coding: utf-8 -*-
"""OFDM 案例：参数表、收发流水线、抓包存取与判定。

与 mod_cases 的分工相同：本模块只做"比特 ↔ 基带 ↔ 判决"，不 import pynq。
硬件收发在 01 与命令行里，用 qpsk_hw.Loopback 一次突发收发。

案例参数
  QPSK 子载波（每符号 2 bit），9 帧 × 1024 bit = 9216 bit
  网格容量每帧 92 子载波 × 6 符号 × 2 bit = 1104 bit（不足部分补 0，不计入 BER）
门限沿用 mod_cases.LIM：evm_max 8%、margin_min 0.85。
"""
import numpy as np

import mod_cases as C
import mod_dsp as M
import ofdm_dsp as O

SPEC = dict(name="OFDM", scheme="ofdm", k=2,
            n_bits=O.N_BITS, frames=9, seed=C.SEED)

LIM = C.LIM


def build(spec=SPEC, peak=C.PEAK, lead=O.LEAD):
    """返回 (const, payload[(tag,bits)], bb, meta)。"""
    const = M.Const("psk", int(spec["k"]))
    payload = C.build_payload_set(int(spec["n_bits"]), int(spec["frames"]),
                                  int(spec.get("seed", C.SEED)))
    bb = O.build_burst([b for _t, b in payload], const, peak=peak, lead=lead)
    meta = dict(name=spec["name"], n_bits=int(spec["n_bits"]),
                frames=len(payload), cap_bits=O.CAP_BITS,
                frame_len=O.FRAME_LEN, n_fft=O.N, cp=O.CP, n_sym=O.N_SYM,
                k_used=len(O.K_USED), k_pilot=len(O.K_PILOT),
                k_data=len(O.K_DATA), lead=int(lead), peak=float(peak),
                papr_db=O.papr_db(bb), burst_len=len(bb))
    return const, payload, bb, meta


def demod(x, fs, bits_list, const, verbose=False):
    """接收缓冲 → 统一结构（与 mod_cases.load_and_demod 的数字分支同形）。"""
    res, sync = O.demod(x, fs, bits_list, const, verbose=verbose)
    m = O.metrics(res, sync, 0, fs)
    return dict(name=SPEC["name"], kind="ofdm", spec=SPEC, res=res,
                sync=sync, metrics=m, x=x)


def save(path, x, bb_len, fs_word, payload, extra=None):
    """抓包 + 真值比特一起落盘，02/03 不依赖随机数重放。"""
    bits = np.concatenate([np.asarray(b, np.uint8).reshape(-1)
                           for _t, b in payload])
    ex = dict(case_kind="ofdm", scheme="ofdm", k=int(SPEC["k"]),
              n_bits=int(SPEC["n_bits"]), frames=len(payload),
              seed=int(SPEC["seed"]), bits=bits,
              n_fft=O.N, cp=O.CP, n_sym=O.N_SYM)
    ex.update(extra or {})
    return C.save_capture(path, SPEC["name"], x, bb_len, fs_word, extra=ex)


def load_and_demod(path, verbose=False):
    """从 01 的抓包复算接收机，供 02/03 用。"""
    z = np.load(path, allow_pickle=False)
    x = M.Q.from_words(z["words"])
    fs = float(z["fs_word"])
    bits = np.asarray(z["bits"], np.uint8)
    n_bits = int(z["n_bits"])
    frames = int(z["frames"])
    bits_list = [bits[i * n_bits:(i + 1) * n_bits] for i in range(frames)]
    const = M.Const("psk", int(z["k"]))
    r = demod(x, fs, bits_list, const, verbose=verbose)
    r["bb_len"] = int(z["bb_len"])
    r["metrics"]["bb_len"] = r["bb_len"]
    r["spec"] = dict(SPEC)
    return r


def verdict(r):
    """返回 (ok, 说明)。门限与单载波案例共用 LIM。"""
    m = r["metrics"]
    ok = (m["err"] == 0 and m["evm_max"] <= LIM["evm_max"]
          and m["margin_min"] >= LIM["margin_min"])
    det = ("BER %d/%d，EVM max %.2f%%（≤%.1f），余量 min %.3f（≥%.2f）"
           % (m["err"], m["tot"], m["evm_max"], LIM["evm_max"],
              m["margin_min"], LIM["margin_min"]))
    return ok, det
