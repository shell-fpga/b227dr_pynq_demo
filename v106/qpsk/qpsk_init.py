# -*- coding: utf-8 -*-
"""qpsk 验证环境的共用初始化：时钟 → 位流 → RFDC → NCO → DMA，一键拉起。

03/04/05 的第一格都调用这里，**不再依赖 01/02 先跑过**。断电重启后的最小用法：

    import qpsk_init as QI
    QI.ensure_clock()             # LMK04828 256 MHz + 连续 SYSREF（断电后必做）
    BIT, HWH = QI.locate_bit()    # 默认用 /home/xilinx/jupyter_notebooks/v106/soc.bit
    lb = QI.open_hardware(BIT)    # Overlay + xrfdc init + NCO + AXI DMA
    QI.probe(lb)                  # 一次收发自检（波峰因数判 ADC I/Q 是否正常）

为什么单靠 notebook 不行：
  * PYNQ 开机不自动下位流，也不自动配 LMK —— RFDC 没有 256 MHz 采样时钟时
    xrfdc.init() 不报错但 tile 状态是 0，现象是"DMA 有 IOC、数据全是 0"；
  * Overlay() 需要同目录同基名的 .hwh，否则能加载但 ip_dict 里没有地址；
  * 下位流会复位 PL，所以必须"先 Overlay、后 xrfdc.init()"，顺序反了初始化被冲掉。
"""
from __future__ import annotations

import glob
import os
import subprocess
import sys
import time

HOME = "/home/xilinx"
JUPYTER_ROOT = "/home/xilinx/jupyter_notebooks"
NB_DIR = os.path.dirname(os.path.abspath(__file__))
LEGACY_DIR = os.path.join(HOME, "qpsk")

TICS = os.path.join(HOME, "LMK04828_256m.tics.txt")
SYSREF_DIV = 384
NCO_MHZ = 1000.0

for _p in (NB_DIR, LEGACY_DIR, HOME):
    if _p not in sys.path:
        sys.path.insert(0, _p)

CHECKS = []


# --------------------------------------------------------------------------- #
# 三态检查表（PASS / FAIL / SKIP），与 02 的判读口径一致
# --------------------------------------------------------------------------- #
def _w(s):
    """终端显示宽度：CJK 与全角符号占两列，直接 len() 会让表格歪掉。"""
    return sum(2 if ord(ch) > 0x2E80 else 1 for ch in s)


def _pad(s, width):
    return s + " " * max(0, width - _w(s))


def check(name, ok, detail="", skip=False, hint=""):
    tag = "SKIP" if skip else ("PASS" if ok else "FAIL")
    CHECKS.append({"name": name, "tag": tag, "detail": detail, "hint": hint})
    print("  [%s] %s %s" % (tag, _pad(name, 44), detail))
    return ok


def reset_checks():
    del CHECKS[:]


def summary(title="汇总", verbose=True):
    rows = list({c["name"]: c for c in CHECKS}.values())
    n = {"PASS": 0, "FAIL": 0, "SKIP": 0}
    for c in rows:
        n[c["tag"]] += 1
    if verbose:
        print("\n" + "=" * 78)
        print(" " + title)
        print("=" * 78)
        print("  %s %s %s" % (_pad("结果", 5), _pad("项目", 44), "实测 / 说明"))
        print("  " + "-" * 76)
        for c in rows:
            print("  %s %s %s" % (_pad(c["tag"], 5), _pad(c["name"], 44), c["detail"]))
        print("  " + "-" * 76)
        print("  PASS %d / FAIL %d / SKIP %d" % (n["PASS"], n["FAIL"], n["SKIP"]))
        bad = [c for c in rows if c["tag"] == "FAIL"]
        if bad:
            print("\n  未通过项与处置：")
            for c in bad:
                print("   - %-44s %s" % (c["name"], c["hint"] or c["detail"]))
    return n


# --------------------------------------------------------------------------- #
# 1. 运行环境
# --------------------------------------------------------------------------- #
def preflight(verbose=True):
    """root / 目录 / 依赖脚本。不碰硬件。"""
    if verbose:
        print("kernel :", os.uname().release)
        print("python :", sys.version.split()[0], "→", sys.executable)
        print("uid    :", os.geteuid())
        print("nb dir :", NB_DIR)
        print("legacy :", LEGACY_DIR)
    reset_checks()
    check("运行身份是 root", os.geteuid() == 0, "uid=%d" % os.geteuid(),
          hint="Jupyter 以 root 启动，否则 /dev/mem 与 spidev 打不开")
    for f in ("lmk04828_readback.py", "lmk04828_set.py", "sysref_start.py"):
        p = os.path.join(HOME, f)
        check("时钟脚本在位 %s" % f, os.path.exists(p), p,
              hint="从出厂镜像复制 /home/xilinx/%s" % f)
    check("256 MHz 寄存器表在位", os.path.exists(TICS),
          os.path.basename(TICS), hint="TICS Pro 导出的 LMK04828_256m.tics.txt")
    return summary("1　运行环境", verbose=verbose)


# --------------------------------------------------------------------------- #
# 2. 时钟：LMK04828 PLL2 + SYSREF
# --------------------------------------------------------------------------- #
def _lmk_open(lrb):
    node = lrb.find_lmk_spi()
    if node is None:
        raise RuntimeError("找不到 compatible=ti,lmk04828 的 SPI 节点")
    dev = lrb.ensure_spidev(node)
    spi = lrb.Spi(dev, speed=500000)
    return spi, lrb.Readback(spi, samples=5)


def _lmk_state(rbk):
    """返回 (读回可信, R0x182, R0x183, R0x139)。读回不可信时这些值不可当真。"""
    sane = False
    try:
        rbk.probe_window()
        rbk.configure()
        sane = bool(rbk.sanity())
    except Exception as e:
        print("  读回异常:", e)
    v182 = (rbk.read(0x182)[0] or 0) & 0xFF
    v183 = (rbk.read(0x183)[0] or 0) & 0xFF
    v139 = (rbk.read(0x139)[0] or 0) & 0xFF
    return sane, v182, v183, v139


def _run(cmd, verbose=True):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    except Exception as e:
        if verbose:
            print("   ⚠ 执行失败:", e)
        return None
    out = (r.stdout or "").strip().splitlines()
    if verbose:
        for ln in out[-6:]:
            print("   ", ln)
        if r.returncode:
            print("   ⚠ 返回码 %d：%s"
                  % (r.returncode, (r.stderr or "").strip().splitlines()[-1:]))
    return r


def ensure_clock(force=False, verbose=True):
    """LMK 没有锁定时自动写 256 MHz 表并启动连续 SYSREF；已锁定则跳过（<0.2 s）。

    返回 dict(sane, ld1, ld2, mux, programmed)，同时登记三项判据。
    """
    if os.geteuid() != 0:
        raise RuntimeError("需要 root：spidev 打不开时无法访问 LMK")
    sys.path.insert(0, HOME)
    import lmk04828_readback as lrb

    spi, rbk = _lmk_open(lrb)
    sane, v182, v183, v139 = _lmk_state(rbk)
    ld1, ld2, mux = (v182 >> 1) & 1, (v183 >> 1) & 1, v139 & 3
    programmed = False

    need = force or (not sane) or (ld2 != 1) or (mux != 3)
    if need:
        if verbose:
            print("→ 时钟未就绪（sane=%s PLL2 LD=%d MUX=%d），写 256 MHz 表 + 连续 SYSREF ..."
                  % (sane, ld2, mux))
        spi.close()
        _run([sys.executable, os.path.join(HOME, "lmk04828_set.py"), "--regs", TICS], verbose)
        _run([sys.executable, os.path.join(HOME, "sysref_start.py"),
              "--sysref-div", str(SYSREF_DIV), "--no-rfdc"], verbose)
        time.sleep(0.5)
        spi, rbk = _lmk_open(lrb)
        sane, v182, v183, v139 = _lmk_state(rbk)
        ld1, ld2, mux = (v182 >> 1) & 1, (v183 >> 1) & 1, v139 & 3
        programmed = True
    elif verbose:
        print("→ PLL2 已锁定、SYSREF 连续输出，跳过写表")
    spi.close()

    reset_checks()
    if verbose:
        print("  LMK 现状: 读回可信=%s  PLL1 LD=%d(0x%02X)  PLL2 LD=%d(0x%02X)  SYSREF MUX=%d"
              % (sane, ld1, v182, ld2, v183, mux))
    check("LMK 读回可信", sane, "R0x171=0xAA / R0x172=0x02",
          hint="读回通路不可信：检查 SPI 速率与节点，必要时 force=True 重写")
    check("LMK PLL2 锁定（RFDC 时钟源）", ld2 == 1, "R0x183=0x%02X" % v183,
          skip=(not sane), hint="PLL2 不锁：先看 sysref_start.py 的输出，再重跑本格")
    check("SYSREF 连续输出（MUX=3）", mux == 3, "R0x139 MUX=%d" % mux,
          hint="sudo %s /home/xilinx/sysref_start.py --sysref-div %d" % (sys.executable, SYSREF_DIV))
    if verbose:
        print("  PLL1 LD=%d（本板 PLL1 空载，LD=0 正常，不是故障）" % ld1)
        summary("2　时钟", verbose=verbose)
    return dict(sane=sane, ld1=ld1, ld2=ld2, mux=mux, programmed=programmed,
                v182=v182, v183=v183)


# --------------------------------------------------------------------------- #
# 3. 位流：默认用板卡 v106/ 下的 soc.bit，必须与同名 .hwh 同源
# --------------------------------------------------------------------------- #
def _bit_candidates():
    out = []
    env = os.environ.get("QPSK_BIT")
    if env:
        out.append(env)
    # 用户口中是"V106"，板上目录是小写 v106；两种大小写都试，Linux 区分大小写
    for d in ("v106", "V106"):
        out.append(os.path.join(JUPYTER_ROOT, d, "soc.bit"))
        out.append(os.path.join(JUPYTER_ROOT, d, "soc_wrapper.bit"))
    out += sorted(glob.glob(os.path.join(JUPYTER_ROOT, "v*", "*.bit")), reverse=True)
    out += sorted(glob.glob(os.path.join(JUPYTER_ROOT, "V*", "*.bit")), reverse=True)
    out.append(os.path.join(JUPYTER_ROOT, "soc_wrapper.bit"))
    out.append(os.path.join(HOME, "soc_test", "soc.bit"))
    out.append(os.path.join(HOME, "soc_wrapper.bit"))
    seen, uniq = set(), []
    for p in out:
        if p not in seen:
            seen.add(p)
            uniq.append(p)
    return uniq


def locate_bit(verbose=True, register=True):
    """返回 (bit, hwh)。.bit 与同名 .hwh 必须同时存在，否则 Overlay 拿不到地址表。"""
    chosen = None
    for b in _bit_candidates():
        h = os.path.splitext(b)[0] + ".hwh"
        if os.path.exists(b) and os.path.exists(h):
            chosen = (b, h)
            break
    if register:
        reset_checks()
        check("位流 soc.bit 在位", chosen is not None,
              chosen[0] if chosen else "未找到（候选见 locate_bit 源码）",
              hint="把 soc.bit + soc.hwh 放到 v106/，或 export QPSK_BIT=/path/soc.bit")
        if chosen:
            check("硬件描述 soc.hwh 同源", os.path.exists(chosen[1]), chosen[1],
                  hint="hwh 必须与 bit 同名同目录")
    if verbose:
        if chosen:
            print("位流 :", chosen[0])
            print("hwh  :", chosen[1])
            print("体积 : %.1f MB" % (os.path.getsize(chosen[0]) / 1e6))
        else:
            print("✗ 没找到配对的 soc.bit + soc.hwh")
    return chosen if chosen else (None, None)


def _on(v):
    return str(v).strip().lower() in ("1", "true")


def dma_info(params):
    """从一个 axi_dma 的 hwh 参数里取出 (方向, MM 位宽, 流位宽, 突发字节, SG)。

    方向只能看 C_INCLUDE_S2MM/MM2S —— hwh 里两条通道的参数**都**存在，
    不看 include 位就会把 MM2S 的 128 错读成 S2MM 的默认值。
    """
    if _on(params.get("C_INCLUDE_S2MM", 0)):
        d = "S2MM/RX"
        mm = params.get("C_M_AXI_S2MM_DATA_WIDTH")
        ax = params.get("C_S_AXIS_S2MM_TDATA_WIDTH")
        burst = params.get("C_S2MM_BURST_SIZE")
    elif _on(params.get("C_INCLUDE_MM2S", 0)):
        d = "MM2S/TX"
        mm = params.get("C_M_AXI_MM2S_DATA_WIDTH")
        ax = params.get("C_M_AXIS_MM2S_TDATA_WIDTH")
        burst = params.get("C_MM2S_BURST_SIZE")
    else:
        d, mm, ax, burst = "?", None, None, None
    nbytes = None
    try:
        nbytes = int(burst) * int(mm) // 8
    except Exception:
        pass
    return dict(dir=d, mm=mm, axis=ax, burst_bytes=nbytes,
                sg=params.get("C_INCLUDE_SG"), params=params)


def hw_summary(hwh, verbose=True, register=True):
    """解析 hwh：IP 清单、两个 DMA 的 MM/流位宽、突发长度、adc2axis 包长。"""
    import xml.etree.ElementTree as ET

    root = ET.parse(hwh).getroot()
    ips, dmas, cfl, fs_word = set(), {}, None, 256.0
    for mod in root.iter("MODULE"):
        full = mod.get("FULLNAME", "")
        parts = [p for p in full.split("/") if p]
        if len(parts) == 2:
            ips.add(parts[1])
        pm = mod.find("PARAMETERS")
        if pm is None:
            continue
        ps = {p.get("NAME"): p.get("VALUE") for p in pm}
        tail = parts[-1] if parts else ""
        if tail.startswith("axi_dma"):
            dmas[tail] = dma_info(ps)
        if "C_FRAME_LEN" in ps:
            cfl = int(ps["C_FRAME_LEN"])

    if verbose:
        print("IP 清单（%d）: %s" % (len(ips), ", ".join(sorted(ips))))
        print()
        print("  %-14s %-8s %-14s %-12s %-10s %s"
              % ("IP", "方向", "MM 数据位宽", "流 TDATA 位宽", "突发字节", "SG"))
        print("  " + "-" * 76)
        for name in sorted(dmas):
            i = dmas[name]
            print("  %-14s %-8s %-14s %-12s %-10s %s"
                  % (name, i["dir"], i["mm"], i["axis"],
                     "-" if i["burst_bytes"] is None else i["burst_bytes"], i["sg"]))
        if cfl:
            print("\n  adc2axis C_FRAME_LEN = %d 字（单包上限 %.3f ms）"
                  % (cfl, cfl / (fs_word * 1e3)))

    if register:
        for name in sorted(dmas):
            i = dmas[name]
            check("%s 存储器侧位宽 = 128" % name, str(i["mm"]) == "128",
                  "C_M_AXI_*_DATA_WIDTH=%s" % i["mm"], skip=(i["mm"] is None),
                  hint="32 位的 DMA 在 256 MHz 下密度 100%，DDR 一有抖动就会产生保持段")
        summary("3　位流 / IP", verbose=verbose)
    return dict(ips=sorted(ips), dmas=dmas, c_frame_len=cfl, fs_word=fs_word)


# --------------------------------------------------------------------------- #
# 4. 硬件对象：Overlay → xrfdc → NCO → DMA
# --------------------------------------------------------------------------- #
def open_hardware(bit=None, nco_mhz=NCO_MHZ, verbose=True, register=True):
    """构造 qpsk_hw.Loopback（内部完成 Overlay 加载 + xrfdc init + NCO + DMA 认方向）。"""
    import qpsk_hw as H

    if bit is None:
        bit, _ = locate_bit(verbose=False, register=False)
    lb = H.Loopback(bit=bit, nco_mhz=nco_mhz, verbose=verbose)

    # DAC 的 state 是 FIFO 满标志：init 刚返回时还在 0x5，数据通路起转后才变 0xF。
    # 板上实测约 100~200 ms 到位；这里最多等 2 s，避免把"正常但还没满"误判成故障。
    st = lb.status()
    for _ in range(20):
        if st.get("DAC", {}).get("state") == 0xF:
            break
        time.sleep(0.1)
        st = lb.status()
    dac, adc = st.get("DAC", {}), st.get("ADC", {})
    if verbose:
        print("  RF 采样率 %.1f MSPS → 字率 %.3f MHz"
              % (lb.fs_rf, lb.fs_word / 1e6))
        for tag, v in (("DAC0", dac), ("ADC0", adc)):
            print("  %s enabled=%s state=0x%X PLL=%s"
                  % (tag, v.get("enabled"), v.get("state", -1), v.get("pll")))
    if register:
        reset_checks()
        check("xRFDC 已就绪", getattr(lb, "rf_ready", False), "is_ready()=%s" % getattr(lb, "rf_ready", None),
              hint="RFDC 不 ready：先确认第 2 节 LMK PLL2 已锁，再调 lb.rf_reinit()")
        check("DAC0 进入 FULL", dac.get("state") == 0xF,
              "state=0x%X" % dac.get("state", -1),
              hint="等 2 s 仍不 FULL：时钟/SYSREF 未就绪，或 tile 被 PLL 关闭")
        check("ADC0 已使能", adc.get("enabled") == 1, "enabled=%s" % adc.get("enabled"))
        check("TX/RX DMA 已定位", lb.TX is not None and lb.RX is not None,
              "TX@0x%08X RX@0x%08X" % (lb.tx_base, lb.rx_base))
        summary("4　RFDC / DMA", verbose=verbose)
    return lb


# --------------------------------------------------------------------------- #
# 5. 收发自检：波峰因数判 ADC 的 I/Q 是否正常
# --------------------------------------------------------------------------- #
CF_CLEAN = 2.5


def probe(lb, n_bits=256, n_frames=2, seed=7, peak=0.45, tries=4, verbose=True):
    """发一小段 QPSK 并抓回，量化波峰因数（干净单边带 ≈1.6，双边带污染 ≥3.4）。

    本板每次下位流后，ADC 有概率只回 I 通道（虚部恒 0），此时包络周期性过零、
    波峰因数升到 3.4 以上，帧定位仍"正常"但解调全错。恢复靠 lb.rf_reinit()。
    """
    import numpy as np
    import qpsk_dsp as D
    import run_test as R

    h = D.rrc_filter(D.SPS)
    frames = R.build_payload_set(n_bits, n_frames, seed)
    bb, lead, spf = D.build_burst([b for _, b in frames], h, D.SPS, peak=peak)
    words = D.to_words(bb)

    cf, x = 0.0, None
    for k in range(tries):
        if k:
            lb.rf_reinit()
        G = lb.loop(words)
        x = D.from_words(G["w"])
        x = x[:G["n"]] if G["n"] else x
        cf = float(D.burst_crest(x, len(bb)))
        if verbose:
            print("  自检 %d%s：收 %d 字，波峰因数 %.2f → %s"
                  % (k, "" if k == 0 else "（重跑 RFDC 初始化）", G["n"], cf,
                     "干净" if cf < CF_CLEAN else "Q 通道异常"))
        if cf < CF_CLEAN:
            break
    reset_checks()
    check("环回波峰因数 < %.1f" % CF_CLEAN, cf < CF_CLEAN, "%.2f" % cf,
          hint="一直 ≥%.1f：先查 SMA 环回线与衰减器，再重下位流" % CF_CLEAN)
    summary("5　收发自检", verbose=verbose)
    return cf, x


# --------------------------------------------------------------------------- #
# 6. 数据文件（03→04→05 之间传 npz）
# --------------------------------------------------------------------------- #
def npz_path(name, prefer_dir=None):
    """优先 notebooks/qpsk 目录，其次旧的 /home/xilinx/qpsk。"""
    d = prefer_dir or NB_DIR
    p = os.path.join(d, name)
    legacy = os.path.join(LEGACY_DIR, name)
    if not os.path.exists(p) and os.path.exists(legacy):
        return legacy
    return p
