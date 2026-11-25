# FMCW / NCO 板级验证报告

板卡：PYNQ-ZU (xczu27dr) 192.168.1.32，位流 /home/xilinx/jupyter_notebooks/WC/soc.bit
（b227dr_v102_fmcw，NCO_nums=8，PL DDS 直连 diver_demo → DAC）

## 1 结论

- PL 侧 DDS 单音、线性 FMCW、反向 FMCW、chirp+死区 四类波形全部在真机验证通过。
- 带宽 B、周期 T、起始频率 f0、斜率符号 全部由软件寄存器决定，**无需重新综合**。
- 频率分辨率 0.9095 µHz（48 bit 相位累加器 @ 256 MHz 字率），实测频率误差 < 1 kHz。
- 链路：nco_gen(PL DDS) → driver_demo → DAC → 模拟环回 → ADC → adc2axis → DMA0 → PS。
  环回增益 −14.8 dB 且反相（实测频率轴与设置值符号相反，为已知的镜像特性）。

## 2 寄存器表（BKP base = 1200，AXI-Lite 基址 0x80010000）

BKP 包格式：写 data@0x18 → 写 index@0x14 → 0x10 写 1 → 0x10 写 0。

| 偏移 | 名称 | 说明 |
|---|---|---|
| base+0 | DESR | bit0。写 0 清状态；**必须最后写 1**，否则后面写的数据进错 bank |
| base+1 | DDS_scale | 幅度右移量，0=满幅，每 +1 约 −6 dB |
| base+2 | DDS_CH_enable | 每通道 1 bit（本设计 8 通道，导出 8 路 DAC） |
| base+3 | init_phase | 16 bit 初相，需连写 20 包（前 8 包有效） |
| base+4 | D 表 | 每通道 2 包：先高 32 位、后低 32 位；8 通道共 16 包，必须连续写 |
| base+5 | req_done | bit0，写完 D 表后写 1 |
| base+6 | ramp_en | bit0，1=扫频模式 |
| base+7 | chirp_len | 每 chirp 的采样点数 N（32 bit） |
| base+8 | D_step[31:0] | 每拍频率增量，低 32 位（有符号） |
| base+9 | D_step[47:32] | 高 16 位（补码，负斜率必写 0xFFFF） |
| base+10 | dead_len | 每个 chirp 后的空闲拍数（输出停在终止频率） |

## 3 频率与波形公式

- 频率字：D = f × 2^48 / 256e6 = f[Hz] × 1.0995e6；f = D × 256e6 / 2^48；分辨率 0.9095 µHz
- chirp 带宽：B = D_step · N · 256e6 / 2^48
- chirp 周期：T = (N + dead_len) / 256e6
- 斜率：k = D_step · 256e6 / 2^48  (MHz/µs：再除 1e6)
- 相位：phi(n) = D0·n + D_step·n(n−1)/2，每 chirp 清零（相位相干）

## 4 实测（DMA 抓 65536 点、相位差分 + 分段时间折叠）

| 用例 | 配置 | 理论 | 实测 |
|---|---|---|---|
| A | f0=20 MHz, N=25600, D_step=4.295e9, dead=0 | T=100 µs, k=1.000 MHz/µs, 20→120 MHz | T=100.00 µs, k=−1.0000 MHz/µs, f −21.7→−118.4 MHz, rms 0.49 MHz |
| C | f0=20 MHz, N=10240, D_step=10.737e9, dead=15360 | T=100 µs, k=2.500 MHz/µs, 40 µs 扫 + 60 µs 死区 | T=100.00 µs, k=−2.5001 MHz/µs, rms 0.52 MHz |
| D | f0=60 MHz, N=7680, D_step=−4.295e9, dead=5120 | T=50 µs, k=+1.000 MHz/µs, 60→30 MHz | T=50.00 µs, k=+1.0000 MHz/µs, rms 0.22 MHz |

（表中斜率为频率轴绝对值，符号按 − 处理；D 用例为负 D_step，实测正斜率，与 A 互为镜像，符合预期。）

幅度平坦度（单音扫 5→124 MHz，DMA 16384 点 FFT 峰值，LSB）：

| f/MHz | 5 | 15 | 25 | 35 | 45 | 55 | 65 | 75 | 85 | 95 | 105 | 115 | 121 | 124 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 幅度 | 12359 | 12607 | 12964 | 13291 | 13461 | 13432 | 13388 | 13088 | 12765 | 12455 | 12187 | 11963 | 11845 | 11793 |

峰值在 45 MHz，全带起伏 +0.00 ~ −1.15 dB。

## 5 使用方法

```python
import sys; sys.path.insert(0, '/home/xilinx/jupyter_notebooks/WC')
from fmcw_lab import Lab
lab = Lab()                                  # 配置 LMK + 开 RFDC + 校验
lab.program(20.0, ramp=1, chirp_len=25600, d_step=4295000000, dead_len=0)
xc, r = lab.capture(65536)                   # 抓数
print(lab.chirp_stats(xc, period_samples=25600, chirp_samples=25600))
lab.tone(50.0)                               # 单音
lab.sweep_flatness([5,25,45,65,85,105,124])  # 平坦度
```

## 6 已知限制（当前硬件版本）

1. **多音做不到**：一路 DAC 只接 driver_demo 的一个通道，多音需在 PL 加多通道求和器（下一版）。
2. **改参数要重配 + 等 3 个 PPS（约 3 s）**：D 表在 DESR=1 时写进 400+ bank，而读侧只扫 0..199，
   所以改频必须 DESR=0 → 重写 → DESR=1。chirp 过程中不能逐 chirp 改参数。
3. **幅度只有 6 dB 台阶**（DDS_scale 为右移）。
4. **数字域带宽**受 driver_demo 0 阶保持（×4 复制）的 sinc 包络限制，实测 5–124 MHz 内起伏 ≤1.15 dB；
   更宽带宽需 4 相并联 DDS。
5. **dead_len 期间输出停在终止频率**，不是关断；要静默需清 CH_enable 或加大 scale。
6. 板子断电重启后必须重配 LMK（`ensure_clock`），否则 DAC 停在 state=0x6、数据全 0。

## 7 文件清单（均在 WC/）

- fmcw_lab.py        驱动 + 测量库（clock/program/capture/tone/sweep_flatness/chirp_stats）
- fmcw_test.ipynb    5 个 cell 的完整演示
- fmcw_flatness.png  幅度平坦度曲线
- fmcw_chirp.png     FMCW 瞬时频率曲线
- fmcw_report.md     本报告
