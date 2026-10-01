# 三方归属与许可

> 本仓库**大量复用**了同济大学 SuperPower 战队的开源成果。
> 我们按 MIT 的要求保留其版权声明，并在此逐项说明。

---

## 1. ⭐ 主要来源：同济 `sp_vision_25`（MIT）

| 项 | 内容 |
|---|---|
| 仓库 | https://github.com/TongjiSuperPower/sp_vision_25 |
| 许可 | **MIT License**（Copyright (c) 2025 TongjiSuperPower） |
| 固定版本 | commit `bd9f5e7`（`main` 分支） |
| 复用了什么 | ⭐ **`tasks/auto_aim/`（43 文件）+ `tasks/auto_buff/`（13）+ `tools/`（26）+ `io/`（48）的算法与实现** —— 详见 `docs/15-同济转换完整性报告.md` |

### ⚠️ 本仓库相对同济**改动了什么**

| 类型 | 说明 |
|---|---|
| **目录架构** | 完全重排（`core/` `utils/` `io/` `drivers/` `src/` `params/` `test/` `scripts/`）|
| **纯修复** | 未初始化 POD（`Armor`/`Lightbar`/`io::Command`/`Target`）、`dm_imu` 的 `<unistd.h>`、`config.hpp` 强制包含、`device` 回退 … |
| **可选优化** | ⭐ **默认全部关闭**，yaml / `--tongji=false` 显式开启才生效 |

### 📌 不包含的内容（需自行获取）

| 内容 | 原因 | 获取方式 |
|---|---|---|
| `assets/`（YOLO 模型 `.xml/.onnx`） | 属同济仓库；且其 readme 注明识别器来自**其它战队开源**，可能有额外条款 | 从同济仓库取，见 README「准备模型」 |
| 相机 / IMU **厂商 SDK**（`.so`/头文件） | ⚠️ **厂商私有许可，不可再分发** | 从海康 / 迈德威视 / 达妙官网下载 |
| `records/`（测试录像） | 体量大且含现场数据 | 自行采集 |

---

## 2. `third_party/serial`（BSD-3-Clause）

| 项 | 内容 |
|---|---|
| 上游 | https://github.com/wjwwood/serial |
| 许可 | **BSD 3-Clause**（可再分发，需保留声明） |
| 用途 | 串口通信（下位机） |

*（本目录保留上游的 `LICENSE`；若缺失请按 BSD-3-Clause 补全声明。）*

## 3. `third_party/tinympc`（来自同济 `sp_vision_25`）

| 项 | 内容 |
|---|---|
| 来源 | 同济仓库的 `tasks/auto_aim/planner/tinympc/`（12 文件，**原样搬运**） |
| 上游 | TinyMPC（https://github.com/TinyMPC/TinyMPC） |
| 许可 | 随同济的 MIT 分发；**TinyMPC 自身的许可请以其仓库为准** |
| 用途 | MPC 的 ADMM 求解器 |

## 4. 构建期依赖（**不随本仓库分发**）

| 依赖 | 许可 | 用途 |
|---|---|---|
| OpenCV | Apache-2.0 | 图像处理 |
| Eigen3 | MPL-2.0 | 线性代数 |
| yaml-cpp | MIT | 配置 |
| spdlog / fmt | MIT | 日志 / 格式化 |
| nlohmann/json | MIT | JSON |
| OpenVINO | Apache-2.0 | YOLO 推理 |
| ROS 2 (rclcpp / std_msgs / sp_msgs) | Apache-2.0 | 哨兵导航桥（可选） |

---

## 5. 致谢

- ⭐ **同济大学 SuperPower 战队** —— 本项目的算法基线，MIT 开源了完整的自瞄/打符实现
- **哈工程 HEU AutoAim** —— 仅作**架构与调试设计**的参考（未复用代码）
- [1] 北京科技大学 Reborn，RM2024 识别训练网络及推理代码开源
- [2] 同济 readme 中的其余引用
