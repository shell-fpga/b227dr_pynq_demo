# b227dr PYNQ demo（RFSoC xczu27dr）

板上（PYNQ 4.0）跑通的两组 notebook + 当前的 DMA/单音/QPSK 验证代码。

## 目录

```
v106/
├── soc.bit                  位流（2026-11-11 16:31:07，SW_CRC=aff100e0，DMA memory-map 128 bit）
├── soc.hwh                  硬件描述，必须与 soc.bit 同名同目录
├── qpsk/                    QPSK 调制解调闭环验证
│   ├── 00_init.ipynb        环境初始化：LMK 256 MHz + SYSREF → v106 位流 → RFDC/NCO → 收发自检
│   ├── 01_qpsk_tx_rx.ipynb  已知比特 → 成型 → DMA 发 → 立即抓回 → qpsk_03_capture.npz
│   ├── 02_qpsk_demod.ipynb  原始缓冲 → 定位/去直流/定时/频偏/锁相环 → 比特 → qpsk_04_demod.npz
│   ├── 03_qpsk_verify.ipynb 真值 vs 解调逐比特比对：BER / EVM / 眼图 / 定时 / 频偏 五条门限
│   ├── qpsk_init.py         初始化流水线（preflight / ensure_clock / locate_bit / hw_summary / probe）
│   ├── qpsk_hw.py           硬件层：Overlay + xRFDC + NCO + AXI-DMA 单次突发收发
│   ├── qpsk_dsp.py          DSP：RRC、ZC 前导、QPSK 映射/判决、帧同步、定时/频偏、EVM
│   └── run_test.py          命令行版回归（多帧长 × 多图样出 BER 表）
└── tone/                    单音链路验证
    ├── 01_module_verify.ipynb
    └── 02_tx_tone_loopback.ipynb
```

## 板上怎么放

Jupyter 的根目录是 `/home/xilinx/jupyter_notebooks`，而 Python 模块放在 `/home/xilinx/qpsk/`（在根目录之外，
必须显式 `sys.path.insert`）：

```bash
sudo mkdir -p /home/xilinx/qpsk
sudo cp v106/qpsk/*.py      /home/xilinx/qpsk/
sudo cp v106/qpsk/*.ipynb   /home/xilinx/jupyter_notebooks/qpsk/
sudo cp v106/soc.bit v106/soc.hwh /home/xilinx/jupyter_notebooks/v106/
```

## 运行顺序

| 步骤 | notebook | 依赖 |
|---|---|---|
| 0 | `00_init` | 无（断电重启后先跑这一本） |
| 1 | `01_qpsk_tx_rx` | 环境已就绪；PL 里已是 v106 则复用不重下 |
| 2 | `02_qpsk_demod` | `qpsk_03_capture.npz` |
| 3 | `03_qpsk_verify` | 上面两个 npz |

`02` / `03` 只读 `.npz`，不碰 PL / DMA / spidev，重启后可以直接跑。

命令行等价物：

```bash
sudo bash -c '. /etc/profile.d/pynq_venv.sh; python3 /home/xilinx/qpsk/run_test.py --quick'
```

少了 `pynq_venv.sh` 会报 `No Devices Found`（PYNQ 4.0 的 pl_server 需要 `XILINX_XRT`）。

## 为什么用 v106 这份位流

TX 侧 DMA 每个时钟要吐一个 32 bit 字（256 MHz × 4 B = 1.024 GB/s）。若 memory-map 数据位宽也是 32 bit，
每条 MM 读都必须在一个时钟内回来，密度 100%，DDR 一有刷新抖动就直接变成 DAC 保持段
（实测每 7.789 µs 出现 41 个字的保持，占 2.06%，RX 峰值从 1.0000 MHz 掉到 0.9756 MHz）。
128 bit 下每 4 个流时钟才要 1 拍 MM 读，密度降到 25%。

自查：

```bash
grep -o 'C_M_AXI_[MS]2MM_DATA_WIDTH" VALUE="[0-9]*"' /home/xilinx/jupyter_notebooks/v106/soc.hwh
```

## 板上实测（2026-11-11）

- 00_init：LMK PLL2 锁定、SYSREF MUX=3、两个 DMA MM 位宽 128、DAC state=0xF、ADC 使能、环回波峰因数 2.07（干净）
- 01 → 02 → 03：9216 比特**零误码**，单帧 EVM 最大 1.50 %，帧间 t0 抖动 ≤0.06 样点，
  残余频偏 110 Hz，最小判决余量 0.96 → 总判定 PASS

notebook 里保留了这一次运行的输出，可直接查看。

## 已知注意事项

- 浏览器里还开着旧 notebook 页面时，自动保存会把内存里的旧版本写回磁盘，覆盖刚上传的文件。换文件后请 `Ctrl+Shift+R` 重载。
- 每次下位流后 ADC 有概率只回 I 通道（虚部恒 0）：波峰因数从 1.6 升到 3.4 以上，帧定位仍正常但解调全错。
  `open_clean_loopback()` / `probe()` 会用 `rf_reinit()` 自动重试。
- 同进程内 `xrfdc` 已 `is_ready()` 时不要再 `deinit/init`，二次 StartUp 会把 ADC tile 拖到 0xC。
