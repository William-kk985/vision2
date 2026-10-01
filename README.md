<div align="center">

# HzMIR Vision

**RoboMaster 视觉系统 · 基于同济 SuperPower `sp_vision_25` 的架构重构**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)]()
[![OpenVINO](https://img.shields.io/badge/OpenVINO-2024.6-0071C5.svg)]()
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E.svg)]()

</div>

---

## 这是什么

一套 RoboMaster **自瞄 + 打符** 视觉系统，覆盖 **步兵 / 英雄 / 哨兵 / 无人机** 四个兵种。

> ### ⭐⭐ 首要原则：**算法源码以同济为准**
> | 维度 | 以谁为准 |
> |---|---|
> | **算法 / 默认行为** | ✅ **同济 `sp_vision_25`**（MIT） |
> | **目录架构 / 模块划分** | 本项目自主设计 |
> | **本项目引入的一切改动** | 🔶 **只算「可选优化」，默认全部关闭** |
>
> 不写任何配置 = **完全同济行为**。

### 本项目的三件事

1. **架构重排** —— 把同济的 `tasks/` + `tools/` + `io/` 重排为
   `core/`（业务角色）· `utils/`（零业务依赖的轮子）· `io/`（通信）· `drivers/`（厂商 SDK）
2. **纯修复** —— 消除未初始化变量、跨静态库的 `inline static`、设备不可用即崩 等
3. **可选优化** —— 一律**默认关闭**，需要时显式开启（见下）

---

## 核心特性

### ⭐ 六层调试体系（替代同济的 `*_debug.cpp`）

| 层 | 机制 | 特点 |
|---|---|---|
| ① L0/L1 常驻 | `core/debug.hpp` 的 `FrameDebug` | 零宏、近零开销 |
| ② 运行期 sink 热插拔 | `SinkHub` + pub/sub | **生产者不知道消费者** |
| ③ 运行期 logger 级别 | 热键 `d` | 不用重编 |
| ④ 编译期宏 | **只剩 `config.hpp` 一处** | `CMakeLists` 强制 `-include` |
| ⑤ `test/` 独立程序 | **20 个 ctest 用例** | 算法级调试 |
| ⑥ 离线可视化 | `scripts/{analyze,plot,compare}.py` | ⭐ **对比 A-B**，零 pandas 依赖 |

### ⭐⭐ 零硬件可验证

四个兵种程序都支持 **录像回放**，没有相机/下位机也能跑通整条链路：

```bash
./src/infantry --video=demo.avi --force-mode=1 --csv=run params/infantry.yaml
python3 scripts/analyze.py run_frames.csv        # 帧预算占用
python3 scripts/plot.py    run_frames.csv --preset plan
python3 scripts/compare.py A_frames.csv B_frames.csv    # A/B 实验对比
```

### ⭐ 可插拔的算法槽位

`core/auto_aim/target/` 有 **3 个策略槽位**（角速度估计 / 过程噪声 / 观测滤波），
共 **7 个实现**；弹道有 **5 个实现**；`utils/wheels/` 收纳 **10 类零业务依赖的轮子**。

---

## 目录结构

```
├── config.hpp            ⭐ 唯一的编译期宏入口（+ 互斥检查）
├── core/                 契约层 + 业务角色
│   ├── types.hpp         ⭐ 零依赖契约（Armor / Target / Command …）
│   ├── debug.hpp         六层调试的数据面
│   └── auto_aim/         detector classifier solver tracker target planner trajectory shooter controller
│   └── auto_buff/        detector solver target planner
├── utils/                轮子（⛔ 不碰业务类型）
│   ├── wheels/           ballistic detect estimate fit track
│   ├── math/             ⭐ chi2.hpp（χ² 分位）
│   ├── ov/               ⭐ device.hpp（OpenVINO 设备回退）
│   ├── ekf/  log/  debug/  config/  concurrency/  control/  yaml/
├── io/                   通信（camera / board / can / ros2）
├── drivers/              厂商 SDK（hikrobot / mindvision / usbcamera / dm_imu）
├── src/                  ⭐ 各兵种独立完整程序（不抽通用 runner）
├── params/               yaml 配置（+ 预生成弹道表）
├── test/                 function / virtual / debug / ros2
├── scripts/              离线分析（Python）
├── tools/scripts/        部署与运维脚本
└── archive/              冻结的模块（含 FROZEN.md 说明为何冻结）
```

---

## 快速开始

### 依赖

| 依赖 | 版本 | 必需 |
|---|---|---|
| CMake / GCC | ≥3.16 / ≥11 | ✅ |
| OpenCV | ≥4.5 | ✅ |
| Eigen3 · yaml-cpp · spdlog · fmt · nlohmann-json | 任意较新 | ✅ |
| OpenVINO | 2024.6 | ✅（YOLO 推理） |
| ROS 2 Humble | — | 🔶 可选（哨兵导航桥） |

```bash
sudo apt install libopencv-dev libeigen3-dev libyaml-cpp-dev libspdlog-dev libfmt-dev nlohmann-json3-dev
```

### ⚠️ 需要自行获取的两样东西

```bash
# ① 模型权重（属同济仓库，本仓库不复制）
git clone --depth 1 https://github.com/TongjiSuperPower/sp_vision_25.git /tmp/sp
ln -s /tmp/sp/assets assets

# ② 相机 / IMU 厂商 SDK —— 许可不允许再分发，请从厂商官网下载后放入：
#    drivers/hikrobot/{lib,include}/   海康 MVS
#    drivers/mindvision/{lib,include}/ 迈德威视
#    （只用 USB 相机 / 只跑录像回放则不需要）
```

### 构建与测试

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
cd build && ctest          # 20 个用例，约 8 s
```

> ⚠️ **Debug 树请务必带 `-DCMAKE_BUILD_TYPE=Debug`** ——
> CMake 会**按目录名硬校验**（名字含 `dbg` 就必须是 Debug），
> 因为 `assert` 在 Release 是空操作，**ctest 结果会失真**。
> 另有 `test_assert_live` 在运行期再次核对。

### 跑起来（零硬件）

```bash
./build/src/infantry --video=demo.avi --force-mode=1 --csv=run params/infantry.yaml
#                                   ↑ 0=IDLE 1=自瞄 2=小符 3=大符
```

---

## ⭐⭐ 可选优化：**默认全关，一键开启**

| 优化 | 同济默认 | 本项目建议 | 如何开启 |
|---|---|---|---|
| 弹道实现 | `ideal`（无阻力斜抛） | `table`（离线 RK4 建表，**0.04 µs 查表**） | yaml 里 `trajectory_impl: table` |
| 射击过滤 | 仅颜色 | + 编号 / 无敌 / 集火（5 条规则） | yaml 里 `armor_filter:` |
| 目标优先级 | 不设 | 4 张表（3/4 号优先等） | yaml 里 `priority_mode: 3` |
| EKF 一致性阈值 | `0.711` | χ²(4) 的 95% 分位 `9.4877` | `--tongji=false` |
| 小陀螺判据 | `ekf_x()[8]` | `ekf_x()[7]`（真正的角速度） | `--tongji=false` |
| 打符推理设备 | 硬编码 `CPU` | 可配 `GPU`/`AUTO` | yaml 里 `model_device:` |

```bash
# 完全同济行为（默认）
./src/infantry params/infantry.yaml
# 启用本项目的优化
./src/infantry --tongji=false params/infantry.yaml
```

⭐ **每一处改动都能枚举、能关闭** —— 详见 `docs/`。

---

## 📊 实测数据（CPU 推理，无 GPU）

| 指标 | 值 |
|---|---|
| 自瞄单帧：YOLO 推理 | **6745 µs（帧预算 67.5%）** |
| 自瞄单帧：感知（取图+编码+预处理） | 2064 µs（20.6%） |
| 自瞄单帧：跟踪 | **3.7 µs（0.04%）** |
| 单帧合计 | **8813 µs（88.1%）** |
| 打符单帧 | **15.6 ms** ⚠️ 超帧周期 |
| RK4 弹道步长 `1e-4 → 1e-3` | 精度 **0 µrad 差异**，快 **10×**；建表 15957 → 6968 ms |

> ⚠️ 以上为**本机 CPU** 数据；换 GPU 会显著下降。**瓶颈 100% 在检测推理**，跟踪只占 0.04%。

---

## ⚠️ 已知问题

| # | 问题 | 状态 |
|---|---|---|
| 1 | `YOLOBase` 无虚析构 → `unique_ptr<YOLOBase>` 删派生对象是 UB | 🔶 来自同济代码，未擅改 |
| 2 | `RhoAdapter::matrices_initialized` 未初始化 | 🔶 主链路**不可达**（`adaptive_rho=0`） |
| 3 | 无敌 ID 映射表（`ENEMY_ID_TO_ARMOR`）与 RM 官方协议**可能不一致** | ⚠️ **待与电控核对** |
| 4 | EKF 的 `R` 矩阵未经真实数据标定 | ⚠️ 需实车录像 |
| 5 | 硬件测试（12 个）与手眼标定工具**未迁移** | 需真机 |

---

## 文档

`docs/` 下有 4 份分析与审计文档（约 900 行）：

| 文档 | 内容 |
|---|---|
| [`12-相对同济的变更审计.md`](docs/12-相对同济的变更审计.md) | ⭐ **每一处改动都可枚举**：27 个文件逐行 diff，13 个与上游逐字相同 |
| [`13-同济功能盘点与覆盖情况.md`](docs/13-同济功能盘点与覆盖情况.md) | 13 个分支**全部评估**，含 `poly` 的真相与 `auto_outpost` 的发现 |
| [`14-同济哨兵与本项目单相机哨兵对比.md`](docs/14-同济哨兵与本项目单相机哨兵对比.md) | `Decider` 10 个方法按「与相机数的关系」分类 |
| [`15-同济转换完整性报告.md`](docs/15-同济转换完整性报告.md) | ⭐ **135/135 源文件核对** · yaml 字段 53/59/63 全一致 · 13→4 主程序 |

⭐ **每一处改动都能枚举、能关闭** —— 这是本项目对上有的最高承诺。

---

## 许可与致谢

⭐ **MIT License** —— 本项目是**同济大学 SuperPower 战队
[`sp_vision_25`](https://github.com/TongjiSuperPower/sp_vision_25)**（MIT）的衍生作品，
其版权声明已按 MIT 要求保留在 [LICENSE](LICENSE) 中。

完整归属、**不含哪些内容、为什么**，见 [THIRD_PARTY.md](THIRD_PARTY.md)。

特别感谢同济大学 SuperPower 战队开源了完整的自瞄/打符实现。
