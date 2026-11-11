# b227dr PYNQ demo（RFSoC xczu27dr）

板上（PYNQ 4.0）跑通的三组 notebook + 当前的 DMA/单音/QPSK/调制案例验证代码。

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
├── modulation/              五个调制案例（ASK / 8PSK / 16QAM / AM / FM）
│   ├── 06_mod_tx_rx.ipynb   参数表 → 星座/波形/频谱 → 逐案例 DMA 发 + 抓回 → mod_06_*.npz
│   ├── 07_mod_demod.ipynb   读 npz 解调：星座、包络、鉴频、EVM → mod_07_demod.npz
│   ├── 08_mod_verify.ipynb  实测指标对照门限，逐案例 PASS / FAIL + 2×2 指标图
│   ├── mod_dsp.py           共用 DSP：Gray 星座、帧同步、判决反馈 PLL、AM/FM 波形与解调
│   ├── mod_cases.py         案例表、基带生成、接收流水线、门限（LIM）、抓包存取
│   └── run_mod_test.py      命令行版五案例回归
└── tone/                    单音链路验证
    ├── 01_module_verify.ipynb
    └── 02_tx_tone_loopback.ipynb
```

## 板上怎么放

Jupyter 的根目录是 `/home/xilinx/jupyter_notebooks`，而 Python 模块放在 `/home/xilinx/qpsk/`
和 `/home/xilinx/modulation/`（在根目录之外，必须显式 `sys.path.insert`）：

```bash
sudo mkdir -p /home/xilinx/qpsk /home/xilinx/modulation
sudo cp v106/qpsk/*.py      /home/xilinx/qpsk/
sudo cp v106/qpsk/*.ipynb   /home/xilinx/jupyter_notebooks/qpsk/
sudo cp v106/modulation/*.py  /home/xilinx/modulation/
sudo cp v106/modulation/*.ipynb /home/xilinx/jupyter_notebooks/modulation/
sudo cp v106/soc.bit v106/soc.hwh /home/xilinx/jupyter_notebooks/v106/
```

`modulation/` 的 notebook 会 `sys.path.insert` 三个目录：自己的 `/home/xilinx/modulation`、
`/home/xilinx/qpsk`（复用 `qpsk_dsp` / `qpsk_hw` / `qpsk_init` / `run_test`）。
只跑 07/08 不用放 `qpsk/` 的 .py 也行，但 06 和命令行回归需要。

## 运行顺序

| 步骤 | notebook | 依赖 |
|---|---|---|
| 0 | `00_init` | 无（断电重启后先跑这一本） |
| 1 | `01_qpsk_tx_rx` | 环境已就绪；PL 里已是 v106 则复用不重下 |
| 2 | `02_qpsk_demod` | `qpsk_03_capture.npz` |
| 3 | `03_qpsk_verify` | 上面两个 npz |
| 4 | `06_mod_tx_rx` | 环境已就绪；LMK 未锁会自动 `ensure_clock()`，失败才回 00 |
| 5 | `07_mod_demod` | `mod_06_*.npz`（5 个） |
| 6 | `08_mod_verify` | `mod_06_*.npz`；会和 07 各解调一遍，互不依赖 |

`02` / `03` / `07` / `08` 只读 `.npz`，不碰 PL / DMA / spidev，重启后可以直接跑。

命令行等价物：

```bash
sudo bash -c '. /etc/profile.d/pynq_venv.sh; python3 /home/xilinx/qpsk/run_test.py --quick'
sudo bash -c '. /etc/profile.d/pynq_venv.sh; python3 /home/xilinx/modulation/run_mod_test.py'
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

## 调制案例（06–08）：ASK / 8PSK / 16QAM / AM / FM

五个案例走**同一条硬件路径**（v106 位流、1000 MHz NCO、DMA 单次突发环回），
共用 `mod_dsp` 里的帧同步与锁相环，分叉只在映射/解调那一层。
这样每个案例的指标差异只反映调制方式本身，而不是链路配置差异。

### 案例参数

| 案例 | 类型 | 映射 | 每符号 | 突发长度 |
|---|---|---|---|---|
| ASK4 | 数字 | 4-ASK，Gray | 2 bit | 9 帧 × (64+512+16) 符号，9216 bit / 43648 样点 |
| 8PSK | 数字 | 8-PSK，Gray | 3 bit | 9 帧 × (64+512+16) 符号，13824 bit / 43648 样点 |
| 16QAM | 数字 | 16-QAM，I/Q 各 Gray | 4 bit | 9 帧 × (64+256+16) 符号，9216 bit / 25216 样点 |
| AM | 模拟 | DSB-LC，μ=0.8 | — | 单音消息 0.5 MHz，2.048 ms 消息 + 前后 4096 样点 |
| FM | 模拟 | 调频，Δf=2 MHz（β=4） | — | 单音消息 0.5 MHz，2.048 ms 消息 + 前后 4096 样点 |

- 数字帧结构沿用 QPSK：`[64 符号 ZC 前导 root=25][载荷][16 符号零保护]`，
  RRC 成型 SPS=8、α=0.35，9 帧拼一个突发，峰值 0.4（−12 dBFS）。
- 模拟案例的载波在基带 8 MHz（相对 1000 MHz NCO），消息是**已知**单音：
  接收端按已知消息做相关对齐后量测，验证的是链路保真度，不是盲解调。
- 门限集中定义在 `mod_cases.LIM`，06/07/08 与命令行回归共用同一份：
  `evm_max=8%`、`margin_min=0.85`、`corr_min=0.98`、`rmse_max=10%`、
  `ratio_tol=0.10`、`ripple_max=10%`。

### 板上实测（2026-11-11）

| 案例 | 关键指标 | 结论 |
|---|---|---|
| ASK4 | BER 0/9216，EVM 2.35%，余量 0.940 | PASS |
| 8PSK | BER 0/13824，EVM 2.24%，余量 0.903 | PASS |
| 16QAM | BER 0/9216，EVM 2.19%，余量 0.863 | PASS |
| AM | corr +0.9996，rmse 3.2%，μ 比值 1.010 | PASS |
| FM | corr +1.0000，rmse 2.2%，Δf 比值 0.977，纹波 2.5% | PASS |

五个案例全 PASS，数字部分零误码。门限相对实测留了 2~3 倍余量：
板上比离线仿真多了 DAC/ADC 量化、环回损耗和时钟抖动，指标会变差，
但不应跨一个量级——跨了就是链路问题，不是调制方式问题。

抓包与离线复算：

```bash
# 板上跑一次并落盘（不落盘就只打表）
sudo bash -c '. /etc/profile.d/pynq_venv.sh; python3 /home/xilinx/modulation/run_mod_test.py --save /tmp/mod_npz'
```

`--save` 存下的 `mod_06_*.npz` 可直接给 07/08 用（5 个文件共约 5 MB，仓库里不提交）；
07/08 只读 npz，不碰 PL / DMA，断电重启后也能跑。

## 已知注意事项

- 浏览器里还开着旧 notebook 页面时，自动保存会把内存里的旧版本写回磁盘，覆盖刚上传的文件。换文件后请 `Ctrl+Shift+R` 重载。
- 每次下位流后 ADC 有概率只回 I 通道（虚部恒 0）：波峰因数从 1.6 升到 3.4 以上，帧定位仍正常但解调全错。
  `open_clean_loopback()` / `probe()` 会用 `rf_reinit()` 自动重试。
- 同进程内 `xrfdc` 已 `is_ready()` 时不要再 `deinit/init`，二次 StartUp 会把 ADC tile 拖到 0xC。
- 命令行跑 `run_mod_test.py` 时用 PYNQ 的虚拟环境解释器
  `/usr/local/share/pynq-venv/bin/python3`，并带上 `XILINX_XRT=/opt/xilinx/xrt`。
  系统 `/usr/bin/python3` 缺 `pydantic`，且找不到 XRT 设备（报 `No Devices Found`）。
  用 `sudo bash -c '. /etc/profile.d/pynq_venv.sh; python3 …'` 最省事。
