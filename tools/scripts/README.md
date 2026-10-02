# `tools/scripts/` —— 编译与启动脚本

> ⭐ 这两个脚本把**踩过的坑**固化进去了 —— 每条规则背后都有一次真实的翻车。

---

## 🚀 快速开始

```bash
# 1) 编译
tools/scripts/build.sh                      # Release（默认，性能用这个）
tools/scripts/build.sh --both --test        # Release+Debug 都编并跑 ctest

# 2) 跑
tools/scripts/run.sh infantry               # 真机：步兵自瞄
tools/scripts/run.sh sentry                 # 真机：哨兵（自动 source ROS2）
tools/scripts/run.sh infantry --video=a.avi --force-mode=1   # 零硬件：录像回放
```

---

## `build.sh` —— 编译

```bash
tools/scripts/build.sh [--release|--debug|--both] [--clean] [--test] [-j N]
```
| 选项 | 作用 |
|---|---|
| `--release` | ⭐ **默认**。性能数据只能用这个 |
| `--debug` | Debug（`assert` 生效，用于测试） |
| `--both` | 两个都编（**跑双构建 ctest 前用**） |
| `--clean` | 删掉构建目录重来（换编译器/依赖时用） |
| `--test` | 编完跑 ctest |

**它自动做的事**：
1. ⭐ **`config.hpp` 变更检测** —— 改了自动重跑 `cmake`（否则 `HAS_ROS2` 等宏**静默不生效**）
2. ⭐ **构建类型纪律** —— 目录名含 `dbg` 就**必须**是 Debug（`assert` 在 Release 是空操作，本项目**栽过 3 次**）
3. **ROS2 自动 source** —— `config.hpp` 开了 `HAS_ROS2` 就 source `/opt/ros/humble` + `ros2_ws/install`
4. **失败给日志路径**，不吞错误

构建目录在**仓库外**：`<workspace>/exp/hzmir_build`（Release）/ `exp/hzmir_dbg`（Debug）。

---

## `run.sh` —— 启动

```bash
tools/scripts/run.sh <infantry|hero|sentry|uav> [程序参数...] [脚本选项]
```
**程序参数**直接透传（`--video=` / `--csv=` / `--record` / `--pj` / `--force-mode=` …）。
用 `run.sh <兵种> --help` 看**程序自己**的参数（会透传给程序）。

| 脚本选项 | 作用 |
|---|---|
| `--watchdog` | 崩了自动重启（走 `watchdog.sh`，指数退避） |
| `--debug-bin` | 强制用 Debug 构建 |
| `--params=<路径>` | 换参数文件 |

**它自动做的事**：
1. ⭐ **构建目录自动选择** —— 优先 Release；只有 Debug 时**警告"性能数字无意义"**
2. ⭐ **校验构建类型** —— 名字暗示 Debug 但实际 Release 就**直接报错退出**
3. ⭐ **哨兵自动修 `ROS_LOG_DIR`** —— `~/.ros/log` 常常**只读**，直接跑会
   `Failed opening file ... Read-only file system`。脚本把它指到 `logs/ros/`
4. ⭐ **哨兵自动 source `sp_msgs`** —— 否则报 `Type support not from this implementation`
5. **tty 检测** —— 非 tty 时**告诉你热键被禁用了**（不会让你白按）

环境变量：`HZMIR_BIN_DIR` 指定构建目录 · `HZMIR_DEBUG=1` 强制 Debug · `HZMIR_WORKSPACE` 换工作区。

---

## 典型工作流

### A. 零硬件验证（**没相机时最常用**）
```bash
tools/scripts/build.sh
tools/scripts/run.sh infantry --video=录像.avi --force-mode=1 --csv=run1
python3 scripts/analyze.py run1_frames.csv      # ⭐ 帧预算占用表
```

### B. 真机调自瞄
```bash
tools/scripts/run.sh infantry
# 终端里按热键：[2] 开 CSV  [1] 开窗口  [4] 存图  [d] 日志级别  [p] 暂停
```

### C. 采集数据（给 `R` 矩阵标定用）
```bash
tools/scripts/run.sh infantry --record --csv=calib
#                             ↑ avi+位姿txt   ↑ 逐帧数据（两个都要开）
```

### D. 崩溃自动重启（赛场）
```bash
tools/scripts/run.sh hero --watchdog
```

### E. 双构建测试（改完代码必做）
```bash
tools/scripts/build.sh --both --test
```

---

## `check-hw.sh` —— 硬件诊断（⚠️ **在你自己终端里跑**）

```bash
tools/scripts/check-hw.sh
```

⭐ **为什么需要它**：「`Not found camera!`」有**两种完全不同的原因**，光看程序日志分不清：

| 现象 | 原因 | 修法 |
|---|---|---|
| `MV_CC_EnumDevices` 返回 **0 个** | ⚠️ **libusb 枚举不到** → `/dev/bus/usb` 不存在 / 权限 | 挂 usbfs + udev 规则 + **重新插拔** |
| `MV_CC_OpenDevice` 失败（有错误码） | 权限够但**被别的进程占着** | 关掉 MVS 客户端 / 别的程序 |

**它检查 7 项**：
1. `lsusb` 里有没有海康（VendorID `2bdf`）
2. ⭐ **`/dev/bus/usb` 挂载**（libusb 的前提）
3. ⭐ **udev 规则**（并提醒「后加的规则必须 reload + 重插」）
4. ⭐ **libusb 实际能不能打开相机**（**关键判定**）
5. 海康 SDK 版本
6. 串口设备 + 权限 + 用户组
7. `/dev/video*`（本项目**不用** V4L2，仅供参考）

> ⚠️ **`Unable to open usb!` 是个误导性日志** —— 它只是 `reset_usb()` 的
> **USB 重置重试**失败，**不代表相机打开失败**。真正要看的是 `Not found camera!`
> （= `MV_CC_EnumDevices` 返回 0）。

---

## `camera-reset.sh` —— 相机复位（海康红灯）

```bash
tools/scripts/camera-reset.sh
```

### 海康相机的灯语

| 灯 | 含义 |
|---|---|
| **蓝灯** | 已上电、**未被取流**（idle，正常待机） |
| ⚠️ **红灯** | **正在取流（grabbing）/ 被占用** |

### 为什么会变红

```
程序 StartGrabbing        → 红灯
正常退出 StopGrabbing+Close→ 蓝灯 ✅
⚠️ StopGrabbing 失败 / handle 未初始化 → **灯不灭**
```

⭐ **本项目 W56 已修**（`capture_stop()` 改成「尽力清理不中途 return」+ 析构显式清理
+ `handle_` 初始化）。**但已经卡住的相机需要本脚本或拔插来复位。**

⭐ 脚本会：① 检查 `lsusb` ② **检查有没有进程占着**（不关掉 reset 也白搭）
③ 用 libusb 做 USB reset ④ 验证。

> ⚠️ **reset 前一定要先关掉占用进程**（MVS 客户端 / 我们的程序），否则 reset 完
> 立刻又被 grabbing → 灯再次变红。

---

## 目录里还有什么

| 文件 | 作用 |
|---|---|
| **`check-hw.sh`** | ⭐⭐ **硬件诊断**（相机 / 串口 / usbfs / udev）—— 见下 |
| **`camera-reset.sh`** | ⭐⭐ **相机复位**（清除海康「红灯常亮」）—— 见下 |
| `watchdog.sh` | 进程崩溃自动重启（搬自同济，改进版：路径自适应 + 指数退避 + 原因记录） |
| `split_video.py` | 切录像（做 A/B 实验用） |
| `build.sh` / `run.sh` | ⭐ 本文件介绍的这两个 |

> 配套：`scripts/*.py` 是**离线分析**（`analyze` / `plot` / `compare`），
> 在**自瞄进程退出后**跑，不占自瞄资源。见 `docs/autoaim_compare/18-Debug体系总览速查.md`。
