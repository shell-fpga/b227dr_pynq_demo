"""QPSK 硬件层：Overlay / xRFDC NCO / AXI-DMA 单次突发收发。

完全沿用 02_tx_tone_loopback.ipynb 已验证的寄存器口径与顺序：
  * 字 = uint32，低 16 位 I、高 16 位 Q（int16）
  * MM2S：reset → RS=1 → 写 SRCADDR → 写 LENGTH（写 LENGTH 才开始搬）
  * S2MM：reset → RS=1 → 写 DADDR →（此时不搬）→ 写 LENGTH → 等 IOC
  * 抓包前先"预热"抓一次整包并丢弃，把 adc2axis 帧计数归零

与 01_module_verify 的联调（reuse）：
  * 位流基准恒定是 v106/soc.bit（128 位 MM）。01 若载入了 soc_test 那类 32 位构建，
    这里会自动切到 v106 —— 以 v106 为准。
  * 若 PL 里已经是同一份位流，用 Overlay(bit, download=False) 只解析 hwh 拿地址表，
    不重下位流（省约 1 s，也不会把 PL 侧已有状态冲掉）。
  * 同一进程内 xrfdc 已经 is_ready() 为真时，绝不再 deinit/init ——
    二次 StartUp 会把 ADC tile 拖到 0xC（01 §3 的实测结论）。
"""
import os
import time

import numpy as np
from pynq import Overlay, MMIO, allocate

JUPYTER_ROOT = "/home/xilinx/jupyter_notebooks"


def pl_bitfile():
    """PL 里当前载入的位流路径。PYNQ 4.0 由 pl_server 统一记账，跨 kernel 进程有效。"""
    try:
        from pynq import PL
        return getattr(PL, "bitfile_name", None)
    except Exception:
        return None


def _same_file(a, b):
    if not a or not b:
        return False
    try:
        return os.path.realpath(a) == os.path.realpath(b)
    except Exception:
        return False


def _bit_label(p):
    """把位流路径缩成 "v106/soc.bit" 这种短标签，打印用。"""
    if not p:
        return "(未知)"
    return "%s/%s" % (os.path.basename(os.path.dirname(p)), os.path.basename(p))


def _resolve_bit():
    """按优先级找配对(bit, hwh)：v106/ → 其余 v*/ → 根目录 → /home/xilinx。
    可用环境变量 QPSK_BIT 覆盖。.hwh 缺一不可，否则 Overlay 拿不到地址表。"""
    import glob
    import os
    cand = []
    if os.environ.get("QPSK_BIT"):
        cand.append(os.environ["QPSK_BIT"])
    cand.append(os.path.join(JUPYTER_ROOT, "v106", "soc.bit"))
    cand.append(os.path.join(JUPYTER_ROOT, "v106", "soc_wrapper.bit"))
    cand += sorted(glob.glob(os.path.join(JUPYTER_ROOT, "v*", "*.bit")), reverse=True)
    cand += [os.path.join(JUPYTER_ROOT, "soc_wrapper.bit"),
             "/home/xilinx/soc_test/soc.bit", "/home/xilinx/soc_wrapper.bit"]
    for b in cand:
        h = os.path.splitext(b)[0] + ".hwh"
        if os.path.exists(b) and os.path.exists(h):
            return b, h
    return cand[0], os.path.splitext(cand[0])[0] + ".hwh"


BIT, HWH = _resolve_bit()

NCO_MHZ = 1000.0
DAC_TILE, DAC_BLK = 0, 0
ADC_TILE, ADC_BLK = 0, 0

SR_W1C = 0x73F0
SR_HALTED, SR_IDLE = 1 << 0, 1 << 1
SR_IOC, SR_ERR = 1 << 12, ((1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) |
                           (1 << 8) | (1 << 9) | (1 << 14))
DMACR_RS, DMACR_RESET, DMACR_IOC_EN = 1 << 0, 1 << 2, 1 << 12


class DmaSimple(object):
    def __init__(self, base, direction, name=""):
        self.name = name or direction.upper()
        self.direction = direction
        self.mmio = MMIO(base, 0x10000)
        if direction == "mm2s":
            self.r_cr, self.r_sr, self.r_addr, self.r_len = 0x00, 0x04, 0x18, 0x28
        else:
            self.r_cr, self.r_sr, self.r_addr, self.r_len = 0x30, 0x34, 0x48, 0x58

    def sr(self):
        return self.mmio.read(self.r_sr) & 0xFFFFFFFF

    def decode(self, v=None):
        v = self.sr() if v is None else v
        bits = ((0, "Halted"), (1, "Idle"), (4, "DMAIntErr"), (5, "DMASlvErr"),
                (6, "DMADecErr"), (7, "SGIntErr"), (8, "SGSlvErr"),
                (9, "SGDecErr"), (12, "IOC"), (13, "Dly"), (14, "Err"))
        on = [n for b, n in bits if v & (1 << b)]
        return ", ".join(on) if on else "(全 0)"

    def reset(self, timeout=0.5):
        self.mmio.write(self.r_cr, DMACR_RESET)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.sr() & SR_HALTED:
                break
            time.sleep(0.002)
        self.mmio.write(self.r_sr, SR_W1C)
        return self

    def start_channel(self):
        self.mmio.write(self.r_sr, SR_W1C)
        self.mmio.write(self.r_cr, DMACR_RS | DMACR_IOC_EN)
        return self

    def stop(self):
        self.mmio.write(self.r_cr, 0)


class Loopback(object):
    """一次 init 之后可反复 tx/rx。"""

    def __init__(self, bit=BIT, nco_mhz=NCO_MHZ, reuse=True, verbose=True):
        self.v = verbose
        self.sentinel = np.uint32(0xDEADBEEF)
        self.bit = bit
        self.reused = False

        t0 = time.time()
        prev = pl_bitfile()
        if reuse and _same_file(prev, bit):
            # PL 里已经是同一份位流 → 只解析 hwh 拿地址表，不重下
            try:
                self.ol = Overlay(bit, download=False)
                self.reused = True
            except Exception:
                self.ol = Overlay(bit)
                self.reused = False
        else:
            self.ol = Overlay(bit)
        if verbose:
            print("  overlay %s %.0f ms，%d 个 IP"
                  % ("承接" if self.reused else "载入",
                     (time.time() - t0) * 1e3, len(self.ol.ip_dict)))
            if not self.reused:
                if prev and not _same_file(prev, bit):
                    print("    PL 里原来是 %s → 按 v106 基准切换" % _bit_label(prev))
                print("    %s（128 位 MM，QPSK 基准构建）" % _bit_label(bit))

        self.rx_base, self.tx_base, self.rf_base, self.cfl = self._locate()
        import xrfdc
        self.xrfdc = xrfdc
        try:
            rf_already = bool(xrfdc.is_ready())
        except Exception:
            rf_already = False
        self.downloaded = not self.reused
        if rf_already and not self.downloaded:
            # 同进程内已 init 过、且位流没重下：再 deinit/init 会把 ADC tile
            # 拖到 0xC（01 §3 的实测结论），直接沿用
            self.rf_state = "承接（同进程已初始化，不重跑 StartUp）"
        else:
            if rf_already:
                # 位流刚被重下，PL 侧 RFDC 逻辑已复位，必须重新走 StartUp
                try:
                    xrfdc.deinit()
                except Exception:
                    pass
            xrfdc.init()
            self.rf_state = "init()" if not rf_already else "deinit+init（位流重下后重建）"
        self.rf_ready = bool(xrfdc.is_ready())
        if self.rf_ready:
            xrfdc.set_fs_mhz(4096.0)
            self.fs_rf = float(xrfdc.fs_mhz())
        else:
            self.fs_rf = 4096.0
        self.fs_word = self.fs_rf / 16.0 * 1e6

        # DMA LENGTH 位宽
        self.len_max = 16777215
        self.max_words = self.len_max // 4

        self.TX = DmaSimple(self.tx_base, "mm2s", "TX")
        self.RX = DmaSimple(self.rx_base, "s2mm", "RX")

        self.nco_mhz = nco_mhz
        self.evt_src = None
        if self.rf_ready:
            self._set_nco()

        self.env = {
            "bit": bit, "bit_label": _bit_label(bit),
            "prev_bit": prev, "prev_label": _bit_label(prev),
            "reused": self.reused, "rf_state": self.rf_state,
            "rf_ready": self.rf_ready, "fs_rf": self.fs_rf,
            "fs_word": self.fs_word, "nco_mhz": self.nco_mhz,
            "c_frame_len": self.cfl,
        }

    # ---------------- 初始化辅助 ----------------
    def _locate(self):
        def find(suffix):
            for k, v in self.ol.ip_dict.items():
                tail = k.split("/")[-1]
                if tail == suffix or tail.endswith(suffix) or k.endswith(suffix):
                    return v
            return None

        def dma(want):
            for k, v in self.ol.ip_dict.items():
                tail = k.split("/")[-1]
                if not tail.startswith("axi_dma"):
                    continue
                p = v.get("parameters", {})

                def on(x):
                    return str(x).strip().lower() in ("1", "true")
                if (want == "mm2s" and on(p.get("C_INCLUDE_MM2S", 0))) or \
                   (want == "s2mm" and on(p.get("C_INCLUDE_S2MM", 0))):
                    return v
            return find("axi_dma_1" if want == "mm2s" else "axi_dma_0")

        rx, tx = dma("s2mm"), dma("mm2s")
        rf = find("usp_rf_data_converter_0")
        adc = find("adc2axis_0")
        cfl = 4194303
        if adc:
            cfl = int((adc.get("parameters") or {}).get("C_FRAME_LEN", cfl))
        return (int(rx["phys_addr"]), int(tx["phys_addr"]),
                int(rf["phys_addr"]), cfl)

    def _set_nco(self, force=False):
        """把 DAC0/ADC0 的 NCO 设到 self.nco_mhz。

        force=False 时先读回现值：已经等于目标就一个字都不写 —— 这样跑过 01 之后
        再进 03，不会因为无谓的重写把 01 的寄存器状态动掉。
        """
        for is_adc, tile, blk, tag in ((0, DAC_TILE, DAC_BLK, "DAC"),
                                       (1, ADC_TILE, ADC_BLK, "ADC")):
            if not force:
                try:
                    cur = float(self.xrfdc.get_nco_mhz(is_adc, tile, blk)[0])
                except Exception:
                    cur = None
                if cur is not None and abs(cur - self.nco_mhz) < 1.0:
                    if self.v:
                        print("  %s0 NCO 已是 %.3f MHz，沿用" % (tag, cur))
                    continue
                if cur is not None and self.v:
                    print("  %s0 NCO %.3f → %.3f MHz（按 QPSK 基准对齐）"
                          % (tag, cur, self.nco_mhz))
            got = None
            for evt in (2, 0, 1, 3, 4):
                try:
                    self.xrfdc.set_nco_mhz(is_adc, tile, self.nco_mhz,
                                           blk=blk, evt_src=evt)
                    got = float(self.xrfdc.get_nco_mhz(is_adc, tile, blk)[0])
                except Exception:
                    continue
                if abs(got - self.nco_mhz) < 1.0:
                    self.evt_src = evt
                    break
            if self.v:
                print("  %s0 NCO 回读 %.3f MHz（evt_src=%s）"
                      % (tag, got if got is not None else -1,
                         self.evt_src if self.evt_src is not None else "?"))

    def rf_reinit(self):
        """只重跑 RFDC 的 deinit/init/NCO，不重下 bitstream（约 0.1 s，比整片重载快 20 倍）。

        用于把"Q 通道回常量"的坏状态刷掉。板上实测：每次下载 bitstream 后，ADC 的
        I/Q 输出有约 1/5 概率变成只有 I（虚部恒为 0），此时接收信号是实信号，解调必然
        崩；重跑 RFDC 初始化有机会恢复。
        """
        try:
            if self.xrfdc.is_ready():
                self.xrfdc.deinit()
        except Exception:
            pass
        self.xrfdc.init()
        self.rf_ready = bool(self.xrfdc.is_ready())
        if self.rf_ready:
            self.xrfdc.set_fs_mhz(4096.0)
            self.fs_rf = float(self.xrfdc.fs_mhz())
            self.fs_word = self.fs_rf / 16.0 * 1e6
            self._set_nco(force=True)
        return self.rf_ready

    def status(self):
        s = {}
        for is_adc, tile, tag in ((0, DAC_TILE, "DAC"), (1, ADC_TILE, "ADC")):
            try:
                st = self.xrfdc.tile_status(is_adc, tile)
            except Exception as e:
                st = {"enabled": -1, "state": -1, "pll": -1, "err": str(e)}
            s[tag] = st
        return s

    # ---------------- 发送 ----------------
    def tx_prepare(self, words):
        """把 words 写进 DMA 缓冲（memcpy + cache flush 耗时都在这里，先做掉）。"""
        buf = allocate(shape=(len(words),), dtype=np.uint32)
        buf[:] = words
        buf.flush()
        self._txbuf = buf
        return buf

    def tx_fire(self):
        """只做寄存器序列，几十微秒量级。"""
        self.TX.reset()
        self.TX.start_channel()
        self.TX.mmio.write(0x18, int(self._txbuf.device_address))
        self.TX.mmio.write(0x28, int(self._txbuf.nbytes))
        return time.time()

    def tx_start(self, words):
        self.tx_prepare(words)
        return self._txbuf, self.tx_fire()

    def tx_wait(self, timeout=0.2):
        t0 = time.time()
        while time.time() - t0 < timeout:
            sr = self.TX.sr()
            if sr & SR_ERR:
                return False, sr, time.time() - t0
            if sr & SR_IOC:
                return True, sr, time.time() - t0
        return False, self.TX.sr(), time.time() - t0

    def tx_free(self):
        try:
            self._txbuf.freebuffer()
        except Exception:
            pass

    # ---------------- 接收 ----------------
    def _rx_alloc(self, n_words):
        b = allocate(shape=(n_words,), dtype=np.uint32)
        b[:] = self.sentinel
        b.flush()
        return b

    def _rx_arm(self, b):
        self.RX.stop()
        self.RX.reset()
        self.RX.mmio.write(self.RX.r_addr, int(b.device_address))
        self.RX.start_channel()

    def _rx_kick(self):
        self.RX.mmio.write(self.RX.r_len, self.req_bytes)

    def _rx_finish(self, b, timeout=1.0):
        t0 = time.time()
        sr, ok = 0, False
        while time.time() - t0 < timeout:
            sr = self.RX.sr()
            if sr & SR_IOC:
                ok = True
                break
            if (sr & SR_HALTED) and not (sr & SR_IDLE):
                break
            time.sleep(0.00005)
        dt = time.time() - t0
        nbytes = self.RX.mmio.read(0x58) & 0xFFFFFFFF
        self.RX.mmio.write(self.RX.r_sr, SR_W1C)
        b.invalidate()
        w = np.asarray(b, dtype=np.uint32).copy()
        b.freebuffer()
        return dict(w=w, sr=sr, ok=ok, n=int(np.count_nonzero(w != self.sentinel)),
                    dt=dt, nbytes=nbytes)

    def rx_words(self):
        """整包请求字数（在 tlast 处结束）与字节数。"""
        n = int(min(self.cfl, self.max_words))
        self.req_words = n
        self.req_bytes = n * 4
        return n

    def loop(self, words, prewarm=True):
        """一次完整收发。

        顺序是 **先起接收、后发突发**（与 02 相反）：
          * 先起接收 → 突发完整落在接收窗内，第 0 帧不会被窗口起点切掉，
            帧序号与发送数据一一对应；02 那套顺序会让窗口起点落在突发中途。
          * 缓冲区在起接收之前就填好，起接收之后只做几个寄存器写，
            所以突发落在窗口开始后 ~100 us 处，整包 16.38 ms 完全够用。
        """
        n = self.rx_words()
        self.tx_prepare(words)             # memcpy + cache flush 先做掉
        if prewarm:
            wb = self._rx_alloc(n)
            self._rx_arm(wb)
            self._rx_kick()
            self._rx_finish(wb, timeout=1.0)

        b = self._rx_alloc(n)
        self._rx_arm(b)
        self._rx_kick()                    # 接收开始流（此时只有直流）
        t_rx = time.time()
        t_tx = self.tx_fire()              # 几十微秒后才开始发
        G = self._rx_finish(b, timeout=1.0)
        tx_ok, tx_sr, tx_dt = self.tx_wait(0.2)
        self.tx_free()
        G.update(gap_us=(t_tx - t_rx) * 1e6, tx_ok=tx_ok, tx_sr=tx_sr,
                 tx_dt=tx_dt, req=n, fs=self.fs_word)
        return G
