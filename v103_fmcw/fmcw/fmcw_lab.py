# fmcw_lab.py -- NCO / FMCW / 多音 实验工具（对应 WC/soc.bit = v103 位流）
# 用法:
#   import sys; sys.path.insert(0, '/home/xilinx/jupyter_notebooks/WC')
#   from fmcw_lab import Lab
#   lab = Lab(); lab.clock(); lab.powerup(); lab.program(20.0)
#   xc, r = lab.capture(); print(lab.tone(xc))
#   lab.multitone([20, 30, 40, 50], amp=4096); print(lab.tone_peaks(lab.capture()[0]))
import time, sys
import numpy as np
from pynq import MMIO

sys.path.insert(0, '/home/xilinx/qpsk')
import qpsk_init as Q

BIT = '/home/xilinx/jupyter_notebooks/WC/soc.bit'
BKP_ADDR = 0x80010000
BASE = 1200
FS_WORD = 256e6
TWO40 = float(2 ** 40)
SR_IOC = 0x1000
NCO_CH = 8

IDX_DESR, IDX_SCALE, IDX_CHEN, IDX_PHASE = 0, 1, 2, 3
IDX_D, IDX_DONE, IDX_RAMP, IDX_CHIRP = 4, 5, 6, 7
IDX_DSTEP_LO, IDX_DSTEP_HI, IDX_DEAD = 8, 9, 10
IDX_AMP_EN, IDX_AMP, IDX_SUM_EN = 11, 12, 13


class Lab:
    # 打开硬件（下载 WC/soc.bit 并初始化 RFDC）
    def __init__(self, bit=BIT, verbose=False):
        self.lb = Q.open_hardware(bit=bit, verbose=verbose)
        self.bk = MMIO(BKP_ADDR, 0x10000)

    # 配 LMK04828 时钟（板子断电后必做）
    def clock(self, force=False):
        return Q.ensure_clock(force=force)

    # 强制重下位流 + 重配 RFDC。上电后若 DAC tile state!=0xF 或抓数全 0，先跑这个
    def powerup(self):
        try:
            self.lb.ol.download()
        except Exception:
            pass
        return self.lb.rf_reinit()

    def _bkp(self, idx, data):
        self.bk.write(0x18, int(data) & 0xFFFFFFFF)
        self.bk.write(0x14, idx)
        self.bk.write(0x10, 1)
        self.bk.write(0x10, 0)

    # 写一套参数。单音: program(20.0)
    # 扫频: program(20.0, ramp=1, chirp_len=25600, d_step=4295000000)
    # 多音: program(freqs=[20,30,40,50], amp=[4096]*4, sum_en=1) 或 multitone()
    # 关系: f = D*256MHz/2^48 ; B = chirp_len*d_step*256MHz/2^48 ; amp 为线性 0..32767
    def program(self, f0_mhz=0.0, ramp=0, chirp_len=0, d_step=0, dead_len=0,
                scale=1, ch_mask=0xFF, wait=3.2, freqs=None, amp=None,
                amp_en=None, sum_en=0):
        b = self._bkp
        if freqs is None:
            f_list = [float(f0_mhz)] * NCO_CH
        else:
            f_list = [float(f) for f in list(freqs)[:NCO_CH]]
            f_list += [0.0] * (NCO_CH - len(f_list))
        if amp_en is None:
            amp_en = 1 if amp is not None else 0
        if isinstance(amp, (list, tuple)):
            a_list = [int(a) for a in list(amp)[:NCO_CH]]
            a_list += [0] * (NCO_CH - len(a_list))
        elif amp is not None:
            a_list = [int(amp)] * NCO_CH
        else:
            a_list = [32767] * NCO_CH
        b(BASE + IDX_DESR, 0)
        b(BASE + IDX_SCALE, scale)
        b(BASE + IDX_CHEN, ch_mask)
        for _ in range(20):
            b(BASE + IDX_PHASE, 0)
        for f in f_list:
            d = int(round(f * TWO40))
            b(BASE + IDX_D, (d >> 32) & 0xFFFFFFFF)
            b(BASE + IDX_D, d & 0xFFFFFFFF)
        b(BASE + IDX_DONE, 1)
        b(BASE + IDX_AMP_EN, 1 if amp_en else 0)
        for a in a_list:
            b(BASE + IDX_AMP, a & 0xFFFF)
        b(BASE + IDX_SUM_EN, 1 if sum_en else 0)
        b(BASE + IDX_RAMP, 1 if ramp else 0)
        b(BASE + IDX_CHIRP, int(chirp_len))
        b(BASE + IDX_DSTEP_LO, int(d_step) & 0xFFFFFFFF)
        b(BASE + IDX_DSTEP_HI, (int(d_step) >> 32) & 0xFFFF)
        b(BASE + IDX_DEAD, int(dead_len))
        b(BASE + IDX_DESR, 1)
        if wait:
            time.sleep(wait)

    # 多音: 通道 0..N-1 各一音，求和后送 DAC0。amp 缺省按 0.9 留余量
    def multitone(self, freqs, amp=None, ch_mask=0xFF, wait=3.2):
        # 未使用的通道幅度置 0，否则它们输出 DC，求和后会把底噪抬高
        n = len(freqs)
        if amp is None:
            a = max(1, int(32767 * 0.9 / max(1, n)))
            amp_list = [a] * n
        elif isinstance(amp, (list, tuple)):
            amp_list = [int(x) for x in list(amp)[:n]]
        else:
            amp_list = [int(amp)] * n
        self.program(freqs=freqs, amp=amp_list, sum_en=1, ch_mask=ch_mask, wait=wait)

    def capture(self, n=16384):
        b = self.lb._rx_alloc(n)
        self.lb.req_bytes = n * 4
        self.lb._rx_arm(b)
        self.lb._rx_kick()
        r = self.lb._rx_finish(b, timeout=0.5)
        w = r['w']
        i = (w & 0xFFFF).astype(np.uint16).view(np.int16).astype(np.float64)
        q = ((w >> 16).astype(np.uint16)).view(np.int16).astype(np.float64)
        return i + 1j * q, r

    @staticmethod
    def tone(xc, fs=FS_WORD):
        n = len(xc)
        x = np.fft.fft(xc * np.hanning(n))
        m = np.abs(x)
        m[0] = 0
        k = int(np.argmax(m))
        f = (k if k <= n // 2 else k - n) * fs / n
        return f, 4 * m[k] / n

    # 多音峰值表: 返回 [(f_Hz, dBFS), ...] 按频率排序
    @staticmethod
    def tone_peaks(xc, n=8, sep=1.5e6, fs=FS_WORD):
        nn = len(xc)
        x = np.fft.fftshift(np.fft.fft(xc * np.hanning(nn)))
        f = np.fft.fftshift(np.fft.fftfreq(nn, 1.0 / fs))
        db = 20 * np.log10(np.abs(x) / (nn / 4.0) + 1e-9)
        out = []
        for i in np.argsort(db)[::-1]:
            if all(abs(f[i] - p[0]) > sep for p in out):
                out.append((float(f[i]), float(db[i])))
            if len(out) >= n:
                break
        return sorted(out)

    def sweep_flatness(self, freqs, n=16384):
        out = []
        for f in freqs:
            self.program(float(f), ramp=0)
            xc, r = self.capture(n)
            if xc.std() < 1:
                out.append((f, None, 0.0))
                continue
            fm, amp = self.tone(xc)
            out.append((f, fm / 1e6, amp))
        return out

    def chirp_stats(self, xc, fs=FS_WORD, guard=400, jump_hz=30e6, period_samples=None, chirp_samples=None):
        ph = np.unwrap(np.angle(xc))
        fi = np.diff(ph) / (2 * np.pi) * fs
        d = np.abs(np.diff(fi))
        idx = [i for i in range(len(d)) if d[i] > jump_hz]
        merged = []
        for i in idx:
            if not merged or i - merged[-1] > 500:
                merged.append(i)
        a = b = None
        if len(merged) >= 2:
            best = max(range(len(merged) - 1), key=lambda i: merged[i + 1] - merged[i])
            if merged[best + 1] - merged[best] > 2000:
                a, b = merged[best], merged[best + 1]
        if a is None:
            if not period_samples:
                return None
            a = merged[0] if merged else int(len(fi) // 4)
            b = a + int(period_samples)
        if b - a < 4 * guard:
            return None
        end = a + int(chirp_samples) if chirp_samples else b
        seg = fi[a + guard:end - guard]
        t = np.arange(len(seg)) / fs
        pf = np.polyfit(t, seg, 1)
        return dict(period_us=(b - a) / fs * 1e6,
                    slope_mhz_us=pf[0] / 1e12,
                    f_start_mhz=float(np.median(seg[:64]) / 1e6),
                    f_end_mhz=float(np.median(seg[-64:]) / 1e6),
                    rms_mhz=float(np.std(seg - np.polyval(pf, t)) / 1e6),
                    n_jumps=len(merged))
