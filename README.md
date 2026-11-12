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
├── AM/ FM/ ASK/ PSK/ QAM/ OFDM/   六个调制案例，每个一套独立的四册
│   ├── 00_<案例>_init.ipynb   上电初始化（时钟/位流/RFDC/环回）+ 本案例基带自检
│   ├── 01_<案例>_tx_rx.ipynb  生成基带 → DMA 发 → 立即收 → mod_01_<案例>.npz
│   ├── 02_<案例>_demod.ipynb  读抓包解调 → mod_02_<案例>.npz（只存指标）
│   └── 03_<案例>_verify.ipynb 真值 vs 解调 → PASS / FAIL 判定表
├── modulation/              全部案例共用的库（notebook 通过 sys.path 引用）
│   ├── mod_dsp.py           DSP：Gray 星座、帧同步、判决反馈 PLL、AM/FM 波形与解调
│   ├── mod_cases.py         案例表、基带生成、接收流水线、门限（LIM）、抓包存取
│   ├── ofdm_dsp.py          OFDM：组帧、训练同步/CFO、LS 信道、导频 CPE、逐帧指标
│   ├── ofdm_case.py         OFDM 案例包装：参数、收发流水线、抓包（含真值比特）、判定
│   └── run_mod_test.py      命令行版五案例回归（不含 OFDM）
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
for c in AM FM ASK PSK QAM OFDM; do
  sudo mkdir -p /home/xilinx/jupyter_notebooks/modulation/$c
  sudo chown xilinx:xilinx /home/xilinx/jupyter_notebooks/modulation/$c
  sudo cp v106/$c/*.ipynb /home/xilinx/jupyter_notebooks/modulation/$c/
done
sudo cp v106/soc.bit v106/soc.hwh /home/xilinx/jupyter_notebooks/v106/
```

六个案例目录里**只放 `.ipynb`**；代码（`mod_dsp.py` / `mod_cases.py` / `ofdm_dsp.py` /
`ofdm_case.py`）和所有 `.npz` 都在库目录 `/home/xilinx/modulation/`，notebook 用
`sys.path.insert` 引用：自己的库目录 + `/home/xilinx/qpsk`（`qpsk_init` / `qpsk_dsp` /
`qpsk_hw` / `run_test`）。案例之间互不依赖：AM 的 01/02/03 只读写 `mod_01_AM.npz` /
`mod_02_AM.npz`，重抓 AM 不影响 FM。

## 运行顺序

| 步骤 | notebook | 依赖 |
|---|---|---|
| 0 | `00_init` | 无（断电重启后先跑这一本） |
| 1 | `01_qpsk_tx_rx` | 环境已就绪；PL 里已是 v106 则复用不重下 |
| 2 | `02_qpsk_demod` | `qpsk_03_capture.npz` |
| 3 | `03_qpsk_verify` | 上面两个 npz |
| 4 | `<案例>/00_<案例>_init` | 冷启动先跑；六个目录各一份，结构同构，只换第 6 节的案例自检 |
| 5 | `<案例>/01_<案例>_tx_rx` | 环境已就绪；LMK 未锁会自动 `ensure_clock()`，失败才回 00 |
| 6 | `<案例>/02_<案例>_demod` | `mod_01_<案例>.npz` |
| 7 | `<案例>/03_<案例>_verify` | `mod_01_<案例>.npz`，重新解调并判定 |

`<案例>` ∈ `AM / FM / ASK / PSK / QAM / OFDM`，每个目录的四册自成一条实验线。
`02` / `03` 只读 `.npz`，不碰 PL / DMA / spidev，重启后可以直接跑。

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

## 调制案例：AM / FM / ASK / PSK / QAM / OFDM（各四册）

六个案例走**同一条硬件路径**（v106 位流、1000 MHz NCO、DMA 单次突发环回），
单载波五案共用 `mod_dsp` 里的帧同步与锁相环，分叉只在映射/解调那一层；
OFDM 用 `ofdm_dsp` / `ofdm_case`，把比特铺到时频网格上，同步改由训练符号完成。
这样每个案例的指标差异只反映调制方式本身，而不是链路配置差异。

每个案例目录内部编号固定为 `00_init → 01_tx_rx → 02_demod → 03_verify`，
四册只靠 `.npz` 传递（`mod_01_<案例>.npz` 抓包、`mod_02_<案例>.npz` 指标），
案例之间没有共享状态：重跑 AM 不会影响 FM / PSK / QAM。

### 案例参数

| 案例 | 类型 | 映射 | 每符号 | 突发长度 |
|---|---|---|---|---|
| ASK4 | 数字 | 4-ASK，Gray | 2 bit | 9 帧 × (64+512+16) 符号，9216 bit / 43648 样点 |
| 8PSK | 数字 | 8-PSK，Gray | 3 bit | 9 帧 × (64+512+16) 符号，13824 bit / 43648 样点 |
| 16QAM | 数字 | 16-QAM，I/Q 各 Gray | 4 bit | 9 帧 × (64+256+16) 符号，9216 bit / 25216 样点 |
| AM | 模拟 | DSB-LC，μ=0.8 | — | 单音消息 0.5 MHz，2.048 ms 消息 + 前后 4096 样点 |
| FM | 模拟 | 调频，Δf=2 MHz（β=4） | — | 单音消息 0.5 MHz，2.048 ms 消息 + 前后 4096 样点 |
| OFDM | 数字（多载波） | QPSK 数据子载波 + BPSK 导频 | 2 bit/子载波 | 9 帧 × 9728 点，9216 bit / 90112 样点 |

- 数字帧结构沿用 QPSK：`[64 符号 ZC 前导 root=25][载荷][16 符号零保护]`，
  RRC 成型 SPS=8、α=0.35，9 帧拼一个突发，峰值 0.4（−12 dBFS）。
- 模拟案例的载波在基带 8 MHz（相对 1000 MHz NCO），消息是**已知**单音：
  接收端按已知消息做相关对齐后量测，验证的是链路保真度，不是盲解调。
- OFDM：N=1024、CP=128（子载波间隔 250 kHz），占用 |k| ∈ [8, 60]（±15 MHz）；
  导频 14 个（|k|%8==0，BPSK）+ 数据 92 个（QPSK）。每帧
  `[训练 ×2][数据 ×6][512 点零保护]`，两段相同训练符号同时给出定时、CFO 与逐子载波
  LS 信道；导频再逐符号纠公共相位。峰值 0.4（PAPR 约 19.5 dB），
  01 抓包时把 9×1024 真值比特一并写进 `.npz`，02/03 不依赖随机数重放。
- 门限集中定义在 `mod_cases.LIM`，06/07/08 与命令行回归共用同一份：
  `evm_max=8%`、`margin_min=0.85`、`corr_min=0.98`、`rmse_max=10%`、
  `ratio_tol=0.10`、`ripple_max=10%`。

### 板上实测（2026-11-12，重组后重跑）

| 案例 | 关键指标 | 结论 |
|---|---|---|
| ASK4 | BER 0/9216，EVM 2.68%，余量 0.925 | PASS |
| 8PSK | BER 0/13824，EVM 2.56%，余量 0.900 | PASS |
| 16QAM | BER 0/9216，EVM 2.40%，余量 0.860 | PASS |
| AM | corr +0.9995，rmse 3.8%，μ 比值 1.012 | PASS |
| FM | corr +1.0000，rmse 2.2%，Δf 比值 0.976，纹波 3.3% | PASS |
| OFDM | BER 0/9216，EVM 1.55%，余量 0.959，CFO −10 Hz | PASS |

六个案例全 PASS，数字部分零误码。门限相对实测留了 2~3 倍余量：
板上比离线仿真多了 DAC/ADC 量化、环回损耗和时钟抖动，指标会变差，
但不应跨一个量级——跨了就是链路问题，不是调制方式问题。

抓包由各案例的 `01_<案例>_tx_rx` 生成（`/home/xilinx/modulation/mod_01_<案例>.npz`），
`02` / `03` 只读它，不碰 PL / DMA，断电重启后也能跑。`.npz` 共约 5 MB，仓库里不提交。

命令行一次跑完五个案例（只打表，不落盘、不参与 02/03）：

```bash
sudo bash -c '. /etc/profile.d/pynq_venv.sh; python3 /home/xilinx/modulation/run_mod_test.py'
```

## 已知注意事项

- 浏览器里还开着旧 notebook 页面时，自动保存会把内存里的旧版本写回磁盘，覆盖刚上传的文件。换文件后请 `Ctrl+Shift+R` 重载。
- 每次下位流后 ADC 有概率只回 I 通道（虚部恒 0）：波峰因数从 1.6 升到 3.4 以上，帧定位仍正常但解调全错。
  `open_clean_loopback()` / `probe()` 会用 `rf_reinit()` 自动重试。
- 同进程内 `xrfdc` 已 `is_ready()` 时不要再 `deinit/init`，二次 StartUp 会把 ADC tile 拖到 0xC。
- 命令行跑 `run_mod_test.py` 时用 PYNQ 的虚拟环境解释器
  `/usr/local/share/pynq-venv/bin/python3`，并带上 `XILINX_XRT=/opt/xilinx/xrt`。
  系统 `/usr/bin/python3` 缺 `pydantic`，且找不到 XRT 设备（报 `No Devices Found`）。
  用 `sudo bash -c '. /etc/profile.d/pynq_venv.sh; python3 …'` 最省事。
