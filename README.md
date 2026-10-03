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
| ① L0/L1 常驻 | **`core/debug_node.hpp`** 的 `FrameDebug` | 零宏、近零开销 |
| ② 运行期 sink 热插拔 | `SinkHub` + pub/sub | **生产者不知道消费者** |
| ③ 运行期 logger 级别 | 热键 `d` + `--log-off` | 不用重编 |
| ④ ⭐ 编译期开关 | **`core/debug.hpp`**（开关与实验总控） | ⭐ **关掉 = 连参数一起消失**（零成本） |
| ⑤ `test/` 独立程序 | **27 个 ctest 用例**（开 ROS2）/ 25（关） | 算法级调试 |
| ⑥ 离线可视化 | `scripts/{analyze,plot,compare}.py` | ⭐ **对比 A-B**，零 pandas 依赖 |

### ⭐⭐⭐ 三层「测试/调试」策略

```
┌─ 正式代码（core/…/<角色>/）──────────── 长期维护，**以同济为准**
│
├─ ⭐⭐ core/debug.hpp（宏 + 重新编译）──── **中间层**：算法的【临时添加 / 替换】
│     · 实现放 `test/function/exp_*.{hpp,cpp}`
│     · 在这儿加一行 `#define HZMIR_EXP_xxx`  ← ⭐ **总控就这一个文件**
│     · 在**相应文件**加装配入口（`#ifdef` 只在装配点）
│     · ⚠️ **CMake 自动解析**（定义宏 + 挂源码）—— 不用手改 CMakeLists
│     · ⚠️ **验证完必须【转正或删除】**（不许长期挂着）
│
└─ test/function/ ─────────────────────── 单一测试 / 较大的改动测试
```

⭐ **为什么需要中间层**：直接在正式代码里手改做实验**很容易忘记改回来**。
放中间层 ⇒ **一行开关，可发现、可一键还原**。

⚠️ **算法只用【编译期宏】，不做运行期热插拔** —— 因为**最终只留一套算法**，
而热插拔会让"当前跑的是哪套"变得不确定，算法对比最怕这个。
（⭐ **但兼容性保留**：实验开关**默认全关** = 完全同济行为。）

⭐ **关掉时是【真·零残留】**（实测用二进制符号验证）：
```
实验 ON  → nm 在 libhzmir_core.a 里【找到】实验符号   ✅
实验 OFF → nm 【找不到】（exp_*.cpp 根本没参与编译） ⬜
```

#### 热键 `3`（PlotJuggler）怎么用

`3` 把每帧的 **20 个字段**打成 JSON，**UDP 发到 `127.0.0.1:9870`**（`--pj=true` 可启动即开）。

##### ① 装 PlotJuggler（⚠️ **别装 ROS 版**）

⚠️ **不要装 `ros-humble-plotjuggler-ros`** —— 那是 **ROS 集成版**（rosbag/topic 插件），
本项目**走 UDP，用不上**，还会拉一堆 `-dev` 包、版本也旧（2.3.1）。

```bash
# ⭐ 首选：官方 .deb（4.0.0，2026-09 发布，不依赖 ROS）
wget https://github.com/PlotJuggler/PlotJuggler/releases/download/4.0.0/plotjuggler4_4.0.0-1_amd64.deb
sudo apt install ./plotjuggler4_4.0.0-1_amd64.deb
#   或者 AppImage（不用装）：chmod +x 后直接跑
#   或者 snap：sudo snap install plotjuggler（官方作者发布，包较大 ~935 MB）
```

##### ② 装 UDP Server 插件（⚠️ **4.x 默认不带！**）

4.0.0 自带的 `data_stream` 只有 `dummy-streamer / foxglove-bridge / plotjuggler-bridge /
ros2-topic-subscriber / webrtc-client` —— **没有 `udp-server`**，不装就收不到数据。

```
PlotJuggler → 菜单里的 Extensions Marketplace → 搜 "UDP Server" → Install → 重启
```
⭐ 装好后落地在 `~/.local/share/PlotJuggler/PlotJuggler4/extensions/udp-server/`
（就一个 `libudp_source_plugin.so` + `manifest.json`，**148 KB**）。

<details><summary>手动装 / 自查有哪些扩展</summary>

```bash
# 官方注册表（列出全部扩展及其下载地址 + sha256）
curl -sL https://raw.githubusercontent.com/PlotJuggler/pj-plugin-registry/refs/heads/development/registry.json
# linux-x86_64 包：
wget https://github.com/PlotJuggler/pj-official-plugins/releases/download/data_stream_udp/v0.9.1/udp-server-0.9.1-linux-x86_64.zip
# ✅ 实测 sha256 = bcd532a1fb45fc33a9ed7a9f3610efaae1d63f947b00b9f3acb7ee2ac6a23c25（与注册表一致）
unzip udp-server-0.9.1-linux-x86_64.zip
cp -r udp-server ~/.local/share/PlotJuggler/PlotJuggler4/extensions/
```
</details>

##### ③ 在 PlotJuggler 里配置并【启动】（⭐ 两步都容易漏，实测走通）

```
① Sources 面板左上三个图标选【📡 流式】（📄=文件 ☁=云）
② 下拉框选 "UDP Server"
③ ⭐ 点下拉框右边的【+】→ 弹出 "UDP Server" 配置对话框
      Transport   udp://          Address  0.0.0.0
      Port        9870            Message Serialization  json
      ☑ Use embedded timestamp field      （字段名默认就是 timestamp，与程序一致）
      Maximum size of arrays: 500   (•)clamp  ( )discard
④ ⭐⭐ 点对话框右下角的【OK】—— **漏了这步，配置不生效，包被直接丢弃**
⑤ ⭐⭐ 点【⏸/▶】把流**启动** —— 图标含义是「点了会做什么」：
        ▶（灰色）= 当前暂停，点它 → 开始
        ⏸（蓝色高亮）= 当前正在播放
⑥ 点数据源名逐层展开（[stream] UDP Server → udp/data）即可看到字段，
   把字段拖到右侧图表区即可画曲线
```

**自检（强烈建议）**：
```bash
ss -lunp | grep 9870        # ⭐ 有输出 = PlotJuggler 真的在监听
# 若已启动但 Value 列全是 '-'，看 socket 队列有没有堆积：
grep -i 268E /proc/net/udp  # 9870=0x268E；⭐ rx_queue 持续非 0 = 收到了但没消费（流没启动）
```

##### ⭐ ④ 跨机器看（笔记本跑自瞄，台式/另一台看图）

⭐ **PlotJuggler 侧默认就支持**（它的 UDP Server 监听 `0.0.0.0`，能收任何来源）。
只要在我们这边指定对方 IP：

```bash
# 在【机器人/小电脑】上跑（把 IP 换成看图那台机器的）
tools/scripts/run.sh infantry --pj=true --pj-host=192.168.1.50 --pj-port=9870
#                              ↑ 开             ↑ 对方 IP            ↑ 默认 9870
```

⭐ **流量极小**（实测）：一帧 JSON **≈ 420 B** → @30fps 仅 **12 KB/s（0.10 Mbps）**
，@150fps 也才 **0.5 Mbps** ⇒ **WiFi 完全够**。

⚠️ **两个注意点**：
| 项 | 说明 |
|---|---|
| **防火墙** | 看图那台要放行 **UDP 9870**（`sudo ufw allow 9870/udp`，或 Windows 防火墙入站规则） |
| **无线丢包** | ⚠️ 无线比有线易丢包 → 曲线会跳。**重要调试建议有线**；UDP 丢包**不重传** |
| **不跨网段** | ⚠️ UDP 广播不穿透路由器；同一局域网内直接用 IP 即可 |

⭐ **自检（在图那台上）**：`ss -lunp | grep 9870` —— 有输出才说明它在等数据。

##### ⑤ 跑起来

```bash
tools/scripts/run.sh infantry --pj=true       # 启动即发（本机）
# 或跑起来后按 3（热插拔，随时开关）
```

⭐ **不装 PlotJuggler 也能验证有没有在发**：
```bash
python3 -c "
import socket,json
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(('127.0.0.1',9870))
d,_=s.recvfrom(65535); o=json.loads(d); print(len(o),'字段:',sorted(o)[:6])"
```

⚠️ **不想折腾插件？** 用按键 `2`（CSV）或 [`tools/csv_viewer.html`](tools/csv_viewer.html) —— **数据完全一样，且不用装任何东西**。

---

#### 各 Debug 通道的开销（实测，供按场景取舍）

> ⚠️ **这些数字不再在启动时打印** —— 放在这里自己查即可。

| 通道 | 自瞄线程 | 后台 | 备注 |
|---|---:|---:|---|
| **窗口**（按键 `1`） | **ms 级 + 阻塞** | — | ⚠️ **唯一会阻塞自瞄线程的**（`cv::imshow`/`waitKey`） |
| **录像**（`--record`） | ~70 ns | **~1.0 核** | MJPG 编码 26.5 ms/帧；实测码率 **1.5 MB/s** |
| **存图**（按键 `4`） | 33 µs（`clone`） | ~0.15 核 | PNG 编码 210 ms/帧；上限 500 张 |
| **CSV**（按键 `2`） | 250 ns | ~0.01 核 | 格式化+落盘在 worker |
| **PlotJuggler**（按键 `3`） | 104 ns | ~0.01 核 | JSON 拼装+`sendto` 在 worker |
| **不挂任何 sink** | ~400 ns | 0 | 仅 L0/L1 常驻 |

⭐ 所有 sink worker 都用 **`SCHED_IDLE`**（零权限）—— **只在 CPU 空闲时运行，不会抢自瞄线程**。

#### 系统层调优（实测结论，避免重复踩坑）

| 项 | 结论 |
|---|---|
| **CPU 绑核** | ⚠️ **不要绑** —— 实测绑核让自瞄**慢 12%~114%**（OpenVINO 也要用那些核）：<br>不绑 9675 µs · 绑 P-core 10809 µs · 绑 E-core 14570 µs · **绑单核 20745 µs** |
| **OpenVINO 线程数** | ⭐ **用默认（自动）** —— 实测 1 线程 16.9 ms / 4 线程 7.3 ms / **自动 5.15 ms（最快）**；设 12 反而 7.39 ms |
| **实时优先级** | ⚠️ 普通用户 `RLIMIT_RTPRIO=0` 设不了 `SCHED_FIFO`；**`SCHED_IDLE` 可以**（worker 已用） |
| **线程总数** | 默认 ~71 个，其中 **~27 个是 OpenVINO 推理池**（`OMP_NUM_THREADS` 可验证）；**我们的 sink worker 默认为 0** |
| **帧预算** | ⭐ **真机 30 fps 相机 → 33 ms/帧**，自瞄处理只用 **~8 ms**（**核少也不掉帧率**） |

### ⭐⭐ 零硬件可验证

四个兵种程序都支持 **录像回放**，没有相机/下位机也能跑通整条链路：

```bash
./src/infantry --video=demo.avi --force-mode=1 --csv=run params/robots/infantry.yaml
python3 scripts/analyze.py run_frames.csv        # 帧预算占用
python3 scripts/plot.py    run_frames.csv --preset plan
python3 scripts/compare.py A_frames.csv B_frames.csv    # A/B 实验对比
```

#### 输出都去哪了（`output/`）

| 子目录 | 内容 | 怎么产生 |
|---|---|---|
| `output/logs/` | `YYYY-MM-DD_HH-MM-SS.log` | ⭐ **每次运行自动**（含 `ros/`） |
| `output/csv/` | `<prefix>_frames.csv` / `_series.csv` | `--csv=<prefix>` 或热键 `2` |
| `output/video/` | `.avi` + **同名 `.txt`**（每帧位姿） | `--record` |
| `output/images/` | `NNNNNN_aim.png` | 热键 `4` |

```bash
ls output/csv/                          # 找 CSV
python3 scripts/analyze.py output/csv/run1_frames.csv
```

⭐ 想换个地方放（比如挂到大盘）：`HZMIR_OUTPUT_DIR=/mnt/ssd/hzmir tools/scripts/run.sh infantry`
⭐ `--csv=/tmp/x/mine` 含 `/` 时**尊重原路径**，不会强制搬到 `output/csv/`。

**磁盘与清理**：启动时打印一行 `output/` 各子目录占用 + 分区剩余；
⭐ **`output/logs/` 超 30 天自动清理**（`--log-keep-days=`），
⚠️ **录像/存图不自动清理**（比赛复盘要用）—— `output/images/` 实测 **~1.3 MB/张**（上限 500 张 ≈ 650 MB）。

#### 没有下位机 / 没有相机时

| 情况 | 行为 | 命令 |
|---|---|---|
| **串口不存在** | ⭐ **自动降级为虚拟下位机**（不再崩） | 直接跑即可 |
| **必须真实下位机** | 没串口就失败退出（同济行为） | `--strict-board` |
| **显式用虚拟板** | 不碰串口 | `--no-board` |
| **只用录像** | 不碰相机、不碰串口 | `--video=录像.avi --force-mode=1` |

```bash
tools/scripts/run.sh infantry --video=demo.avi --force-mode=1   # 录像回放
tools/scripts/run.sh infantry --no-board                        # 显式虚拟板
tools/scripts/run.sh infantry --strict-board                    # 没下位机就失败
# --force-mode: 0=IDLE 1=自瞄 2=小符 3=大符
```

⚠️ **虚拟下位机的限制**：IMU 姿态恒为单位四元数、弹速取 yaml 配置值
⇒ **EKF 的 yaw/ω 预测、弹道误差、命中判定都不可信**；但**检测/跟踪/解算/规划的内部量可以照常看**。

#### ⭐⭐ 录制-回放离线调参（`tune.sh`）

⭐ **录一次，反复回放调参** —— 不用每次改参数都上真机：

```bash
# ① 上真机录一段（20 秒）
tools/scripts/tune.sh record infantry --name=spin_test --secs=20

# ② 之后【反复】回放调参（核心价值）
tools/scripts/tune.sh replay spin_test --tongji=false
tools/scripts/tune.sh replay spin_test --nis-thresh=chi2
tools/scripts/tune.sh replay spin_test --stop-after=detect    # 只看纯检测率

# ③ 对比两次
CSV_PREFIX=output/csv/base  tools/scripts/tune.sh replay spin_test >/dev/null
CSV_PREFIX=output/csv/tuned tools/scripts/tune.sh replay spin_test --tongji=false >/dev/null
tools/scripts/tune.sh compare base tuned
```

**其他子命令**：
| 命令 | 作用 |
|---|---|
| `list` | 列出已录的段（名字/大小/位姿行数） |
| `analyze <名字>` | 找（或生成）CSV 并出报告 |
| `clean --days=7` | 清理旧录制（⭐ **.avi + .txt 整组删，不留孤儿**） |

⭐ **环境变量**：`ROBOT=hero`（换兵种）· `CSV_PREFIX=xxx`（CSV 前缀）

⭐ **`compare` 输出示例**（计数型看非零帧占比，数值型看均值）：
```
  指标                            base         tuned            变化
  det_armor_count              75.7%         75.7%         +0.0pp
  det_best_conf                0.681         0.681         +0.0%
  t_frame_us               10403.689     10930.818         +5.1%   ← ⭐ 一眼看出变慢
```

---

#### ⭐⭐ YOLO 能看哪些关键信息（**两层：概览 vs 细节**）

⭐ **分层原则**（W106 定）：
```
概览（大概情况） → 运行期可看：logger 级别（热键 d）/ --log-only / CSV / PlotJuggler
细节（内部量）   → 编译期宏：core/debug.hpp §一 的 HZMIR_LOG_YOLO
```

**① 概览 —— 不用重编就能看**

```bash
# 只看 yolo 相关（⭐ 大小写不敏感 + 前缀匹配 ⇒ yolo/YOLO/yolov5 都行）
tools/scripts/run.sh infantry --log-only=yolo
# 静音它
tools/scripts/run.sh infantry --log-off=yolo
```

⭐ **日志**（**已节流**：无候选每 60 帧一条、输出统计每 30 帧一条 ⇒ 不会刷屏）：
```
[YOLOV5] 本帧无候选：objectness 峰值 0.002 < 阈值 0.70
         （采样 25200 个 anchor；>0.3 有 0 个、>0.1 有 0 个）      ← ⭐ "有没有戏"一眼看出
[yolov5] objectness 通过 6 → 输出 1（滤掉 not_armor 0 / conf 0 / type 0）
[yolov5] objectness 通过 2 个候选 → 全被滤掉：not_armor 0 / 置信度 1 / 类型不符 0
         ↑ ⭐ "卡在哪一步"（每 30 帧一条）
```

⭐ **数值（`FrameDebug` → CSV / PlotJuggler，**不用重编、能看趋势**）：

| CSV 列 | 含义 | 怎么看 |
|---|---|---|
| ⭐ **`det_obj_peak`** | **objectness 峰值** | ⭐ **"为什么没检测到"的直接答案**：跟 `det_score_thr` 并排比 |
| **`det_score_thr`** | 当时的 objectness 门槛 | 差多少一眼看出 |
| `det_nms` | NMS 后存活数 | 候选多不多 |
| `det_armor_count` | 输出装甲板数 | 最终结果 |
| `det_best_conf` | 最高置信度 | 过了 objectness 之后的质量 |
| `det_t_infer_us` | 推理耗时 | 帧预算 |

⭐ **怎么用它**（`assets/demo` 687 帧的**真实分布**实测）：
```
det_obj_peak 分布：中位 0.876 · 均值 0.801
  [0.70, 1.01)  585 帧  ████████████████████████████████████████  ← 85% 在这儿（正常）
  [0.50, 0.70)   30 帧  ██
  [0.30, 0.50)   19 帧  █
  [0.10, 0.30)   27 帧  █
  [0.01, 0.10)   25 帧  █
  [0.00, 0.01)    1 帧                                            ← ⚠️ 极端暗帧
```
⇒ ⭐ **判读**：
| `det_obj_peak` | 含义 | 怎么办 |
|---|---|---|
| **> 0.7** | 模型"看到了东西" | 后面被滤掉 ⇒ 查 `conf`/`type` 分解 |
| **0.3 ~ 0.7** | 勉强 | 光照/对焦有问题 |
| ⭐ **< 0.1** | ⚠️ **模型认为完全不像装甲板** | **镜头/曝光/对焦问题**（软件提亮也没用） |

⚠️ **本文件早期版本写"能识别的帧 ≈ 0.1~0.27"—— 那个数字是错的**（W106 实测修正：
真实值 ≈ **0.80**。当时把"某几帧"当成了"全部"）。

---

#### ⭐⭐ 三个检测器怎么对比（v5 / v8 / v11）

⭐ **同一次录像分别跑，落三份 CSV 再比**（`yolo_name` 是**运行期**选的，不用重编）：
```bash
for m in yolov5 yolov8 yolo11; do
  sed "s/^yolo_name:.*/yolo_name: $m/" params/robots/infantry.yaml > /tmp/y_$m.yaml
  tools/scripts/run.sh infantry --video=demo.avi --force-mode=1 \
      --csv=output/csv/yolo_$m /tmp/y_$m.yaml
done
python3 scripts/compare.py output/csv/yolo_yolov5_frames.csv \
                            output/csv/yolo_yolov8_frames.csv   # ⚠️ 收【完整 CSV 路径】
```

⭐ **实测（`assets/demo` 687 帧，本机 CPU）**：
| 检测器 | 出装甲板 | `obj_peak` 均值 | 耗时均值 | anchor 数 |
|---|---|---|---|---|
| **yolov5**（默认） | **520/687** | 0.801 | **5805 µs** | 25200 |
| **yolov8** | 333/687 | 0.804 | 6516 µs | 3549 |
| **yolo11** | 427/687 | 0.733 | ⚠️ **12617 µs** | 8400 |

⚠️ **注意**：这是**同一份录像**的对比 ⇒ 相对关系有意义，**绝对帧率仍以真机为准**。
⭐ 想换默认检测器 ⇒ 改兵种 yaml 的 `yolo_name`（**同济也是这么选的**，不是运行期通信）。

**② 细节 —— 编译期宏（要重编）**

```cpp
// core/debug.hpp §一
#define HZMIR_LOG_YOLO     // ⭐ 逐帧的 anchor 分布 / 各阶段丢弃明细
```
⚠️ 开了**每帧都打**，只适合"盯着某一帧看"。

---

#### ⭐⭐⭐ 日志太吵？—— **编译期开关**（推荐）

⭐ **所有细节日志的开关都在一个文件**：**`core/debug.hpp`**（开关与实验总控，§一）

```cpp
// core/debug.hpp —— 一行一个模块，`#define` = 开 / 注释 = 关
// #define HZMIR_LOG_YOLO       // YOLO 逐帧候选/过滤统计（⚠️ 每帧 1~2 条，最吵）
// #define HZMIR_LOG_EKF        // EKF 新息（NIS）/ 收敛 / 发散
// #define HZMIR_LOG_TRACKER    // 跟踪状态机切换
// #define HZMIR_LOG_PLANNER    // MPC 迭代/弹道
// #define HZMIR_LOG_BUFF       // 打符细节
// ...
```

**用法**（⭐ **源码里不需要 `#ifdef`**）：
```cpp
LOG_YOLO("objectness 通过 {} → 输出 {}", n_pass, n_out);
LOG_EKF("[Target] r={:.3f}, l={:.3f}", ekf_x[8], ekf_x[9]);
```

### ⭐ 为什么用宏而不是运行期过滤（实测数据）

| 方式 | 成本（含昂贵参数） | 说明 |
|---|---|---|
| ⭐ **编译期宏（默认关）** | **1.65 ns/次** | ⭐ **与纯循环基准（1.78 ns）持平 = 零成本** |
| ⚠️ 运行期过滤 `--log-off` | **89.44 ns/次** | 参数**照样求值** + 级别检查 |

**⇒ 54× 差距**。原因：宏关掉时**整条语句连参数一起消失**；运行期过滤只挡住"打印"，**参数已经算完了**。

⭐ **实测效果**（录像 687 帧）：
```
默认（全关）        : debug 日志  9 条（只剩启动期一次性）
打开 HZMIR_LOG_YOLO : yolo 日志 144 条
```

### ⚠️ 运行期过滤还留着吗？

**留着**（`--log-off` / `--log-only`）—— 它适合「临时试一下不想重编」。
⚠️ 但**不要长期依赖**：有 89 ns/次的代价，且 ⚠️ **热键 `n` 已移除**（用户反馈"很难记忆"）。

### ⭐ 怎么加一个新模块

1. **`core/debug.hpp`** 的 §一 加一行 `#define HZMIR_LOG_你的模块`
2. 加一对宏：
   ```cpp
   #ifdef HZMIR_LOG_你的模块
   #  define LOG_你的模块(...) tools::logger()->debug("[tag] " __VA_ARGS__)
   #else
   #  define LOG_你的模块(...)   // ⭐ 展开为空 ⇒ 零成本
   #endif
   ```
3. 源码里把 `tools::logger()->debug("[tag] …", …)` 换成 `LOG_你的模块("…", …)`

---

#### ⭐⭐ 调相机参数（不用每次开 MVS）

⭐⭐ **两层，按【谁的东西】分**（W102 重划 —— 原来把兵种参数塞进 `cameras/` 是**分层放错了**）：

```
params/
├── ⭐ cameras/          ← **只放【品牌 / 设备】级配置**（换相机才动）
│   ├── hikrobot.yaml     海康：VID:PID、PixelFormat、白平衡、通用 camera_params
│   ├── mindvision.yaml   迈德威视：gamma
│   └── usbcamera.yaml
│
└── ⭐ robots/           ← **兵种的【全部】配置**（含它的相机参数）
    ├── infantry.yaml     camera_name=hikrobot / 曝光 10ms / gain 16 / 标定 / 算法参数
    ├── hero.yaml         camera_name=hikrobot / 曝光 2ms
    ├── sentry.yaml       camera_name=hikrobot / 曝光 0.8ms / gain 16.9
    └── uav.yaml          ⭐ camera_name=**mindvision** / 曝光 8ms / gamma 0.6
```

**兵种 yaml 里的相机段**（就这几行，一眼能改）：
```yaml
#####----- ⭐⭐⭐ 相机（本兵种特有）-----#####
camera_name: "hikrobot"      # ⭐ 品牌 → 自动带出 params/cameras/hikrobot.yaml
vid_pid: "2bdf:0001"         # ⭐ 插了多台时按它选
exposure_ms: 10              # 曝光（ms）—— ⚠️ 画面暗先加这个
gain: 16                     # 增益（dB）
fps: 30
```

⭐ **优先级（低 → 高）—— 只有两层**：
```
params/cameras/<品牌>.yaml   <   params/robots/<兵种>.yaml
      （品牌默认）                  （兵种覆盖，也是唯一要改的地方）
```
⭐ **品牌文件怎么找到的**：由`camera_name` 推出 `params/cameras/<品牌>.yaml`
（**uav 写 `mindvision` 就自动加载 `cameras/mindvision.yaml`**）。
⚠️ 不依赖当前工作目录 —— 是**跟着兵种 yaml 的位置**找（`params/robots/x.yaml → params/cameras/<品牌>.yaml`）。

**想改什么 → 改哪**：
| 想改什么 | 改哪 |
|---|---|
| ⭐ **某兵种的曝光 / 增益 / 帧率** | **`params/robots/<兵种>.yaml`** |
| ⭐ **换相机品牌** | 同上，改 `camera_name`（品牌文件自动跟着换） |
| **海康通用参数**（白平衡 / 像素格式 / Gamma） | `params/cameras/hikrobot.yaml` |
| **临时试一组值**（不改文件） | CLI：`--print-config` 看当前；`--camera-config=<path>` 覆盖品牌文件 |

**换另一个海康相机也不会崩**（W93 修的两处）：
| 原来 | 现在 |
|---|---|
| ⚠️ **永远取 `pDeviceInfo[0]`** → 插两个时随机选 | ⭐ **按 `vid_pid` 匹配**，并打印「选中设备 #1/2：`xxxx:xxxx` 型号… 序列号…」 |
| ⚠️ **`type_map.at()`** → 像素格式不是 Bayer8 就**抛异常崩** | ⭐ **`find()` + 明确报错 + 跳过该帧**（不崩），并提示怎么改 `PixelFormat` |

⭐ **通用通道（任意海康参数）** —— 放**品牌文件**（那是"这个品牌特有的"）：
```yaml
# params/cameras/hikrobot.yaml
camera_params:
  float:                      # MV_CC_SetFloatValue
    Gamma: 1.0
    Sharpness: 50
    ExposureTime: 6000        # µs（⚠️ 单位与 exposure_ms 不同）
  enum:                       # MV_CC_SetEnumValue
    BalanceWhiteAuto: 0       # ⭐ 0=Off / 1=Once / 2=Continuous（默认 2，会漂）
  int:                        # MV_CC_SetIntValue
    Width: 1280
```
⚠️ 某个兵种要**单独**改某一项 ⇒ 写进**它自己的 yaml**（同一个 `camera_params` 结构，兵种级覆盖品牌级）。

⭐ **不知道参数名 / 该填多少？先 dump 当前值**：

```bash
tools/scripts/run.sh infantry --dump-camera-params
# 打印 ExposureTime / Gain / Gamma / Sharpness / BalanceWhiteAuto / PixelFormat …
# 的【当前值 + 取值范围】
```

**推荐流程**：
```
① 在 MVS 里把画面调好（曝光/增益/白平衡/Gamma…）
② 跑 `--dump-camera-params` 把那些值打出来
③ ⭐ 抄进 params/robots/<兵种>.yaml（兵种自己的）或 params/cameras/<品牌>.yaml（品牌通用的）
④ 以后直接跑我们的程序，不用再开 MVS
```

⭐ **想确认"到底生效的是哪个值"**：
```bash
tools/scripts/run.sh infantry --print-config params/robots/infantry.yaml
# → ③ 相机会打出【两层合并后】的 camera_name / vid_pid / exposure_ms / gain / fps
```

⚠️ **注意**：
| 项 | 说明 |
|---|---|
| `camera_params` **只对 `hikrobot` 生效** | uav 用的是 `mindvision`（参数走 `gamma`/`exposure_ms`） |
| **应用顺序** | 默认参数 → `camera_params`（**可覆盖**上面的 `exposure_ms`/`gain`） |
| **参数名写错** | 会打 `MV_CC_SetXxxValue(...) failed: 0x...` 的**警告**，不会崩 |

---

#### 关于 `/dev/video*`

⚠️ 本项目用**工业相机 SDK**（海康 `MV_CC` / 迈德威视），**不是 V4L2**。
插着 USB 摄像头时 `ls /dev/video*` 有设备是**正常的**，与本项目无关。
若程序报 `Not found camera!` → 那是 **SDK 没找到相机**，**不是** `/dev/video*` 的问题
（排查：相机供电/USB 带宽/MVS 客户端是否占用）。

#### 串口权限

```bash
sudo usermod -a -G dialout $USER      # 加组后需重新登录
udevadm info -a -n /dev/ttyACM0 | grep -E '({serial}|{idVendor}|{idProduct})'   # 拿 ID 写 udev 规则
```

### ⭐ 可插拔的算法槽位（**运行期**选，不用重编）

| 槽位 | yaml 键 | 可选值 |
|---|---|---|
| **弹道** | `trajectory_impl` | `ideal`（同济默认）/ `rk4` / `rk4_drag` / `rk4_42` / `table` / `table_42` / `tongji_linear(_log)` |
| ⭐ **检测器** | `detector_impl` | `yolo`（默认）/ `traditional`（传统灯条配对） |
| **YOLO 版本** | `yolo_name` | `yolov5`（默认）/ `yolov8` / `yolo11` |
| **目标优先级** | `priority_mode` | 4 张表 |
| **射击过滤** | `armor_filter` | 跳过名单 / 无敌 / 集火 |

⭐ **`--tongji`（默认 `true`）**集中控制**源码里硬编码**的偏差；`--tongji=false` 启用本项目优化：
```bash
--tongji=false                    # 两项都切到"修正"值
--nis-thresh=chi2                 # ⭐ 只改 EKF 一致性阈值（9.4877），判据保持同济
--yaw-rate-src=x7                 # ⭐ 只改小陀螺判据（用 x[7]=w），阈值保持同济
```
⭐ **单项覆盖 > 总开关 > 内置默认（同济）** ⇒ **能单独归因**了。

⚠️ **和 §④ 编译期开关的分工**：这里是「**配置**」（长期、可运行期切）；
`core/debug.hpp` 的 `HZMIR_EXP_*` 是「**实验**」（临时、验证完就删）。

`core/auto_aim/target/` 另有 **3 个策略槽位**（角速度估计 / 过程噪声 / 观测滤波）共 **7 个实现**；
`utils/wheels/` 收纳 **10 类零业务依赖的轮子**。

---

## 目录结构

```
├── output/               ⭐⭐ **所有运行期产物**（已 gitignore；可用
│   ├── logs/              `HZMIR_OUTPUT_DIR` 改根目录）
│   ├── video/             日志（含 ros/）
│   ├── images/            录像 .avi + 同名 .txt 位姿
│   └── csv/              存图 .png
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
├── params/               ⭐ yaml 配置：`cameras/`(品牌) + `robots/`(兵种) —— 见 params/README.md
├── test/                 function / virtual / debug / ros2
├── scripts/              ⭐ 业务 Python（只放 .py）：CSV 分析 / A-B 对比 / 画图
├── tools/                ⭐ 调试工具（不限语言）：csv_viewer.html / diagnose_frame.py / split_video.py
│   └── scripts/          ⭐ 只放 sh：build / run / tune / watchdog / camera-reset / check-hw
├── docs/                 ⭐ 审计与设计文档（索引见 docs/README.md）
└── archive/              冻结的模块（含 FROZEN.md 说明为何冻结）
```

### ⭐ `scripts/` · `tools/` · `tools/scripts/` 怎么分工（别放错）

| 目录 | 放什么 | 语言 | 判据 |
|---|---|---|---|
| ⭐ **`tools/`** | **调试工具** | **不限**（py / html） | 我为了**看 / 查 / 诊断**而跑的 |
| **`tools/scripts/`** | 构建 / 运行 / 硬件脚本 | ⭐ **只 sh** | 执行一组命令用的 |
| **`scripts/`** | **跟项目业务强相关的 py** | ⭐ **只 py** | 它处理**项目的数据 / 算法 / 指标** |

⭐ **各目录有自己的 `README.md` 写明定位** —— 加文件前先看一眼。

---

## 快速开始

### 依赖

| 依赖 | 版本 | 必需 |
|---|---|---|
| CMake / GCC | ≥3.16 / ≥11 | ✅ |
| OpenCV | ≥4.5 | ✅ |
| Eigen3 · yaml-cpp · spdlog · fmt · nlohmann-json | 任意较新 | ✅ |
| OpenVINO | 2024.6 | ✅（YOLO 推理） |
| ROS 2 Humble | — | 🔶 可选（**哨兵导航桥**）<br>⭐ 本机 `ros2_ws/install/sp_msgs` **已在** ⇒ 自动启用 |

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
cd build && ctest          # ⭐ 27 个用例（开 ROS2）/ 25 个（关），约 10 s
```

> ⚠️ **Debug 树请务必带 `-DCMAKE_BUILD_TYPE=Debug`** ——
> CMake 会**按目录名硬校验**（名字含 `dbg` 就必须是 Debug），
> 因为 `assert` 在 Release 是空操作，**ctest 结果会失真**。

⭐ **ROS2 与测试数**：`CMakeLists.txt` 会**自动探测** `sp_msgs`，
探测到就默认开 `HZMIR_WITH_ROS2`（测试数 **25 → 27**，多 2 个 ROS2 往返用例）。
```bash
# 强制开关
cmake -S . -B build -DHZMIR_WITH_ROS2=ON      # 或 OFF
```
⚠️ **`tools/scripts/build.sh` 会替你 source** `/opt/ros/humble` + `ros2_ws/install`（W105 修复：
原来读的是**已删除的** `config.hpp` ⇒ **永远不 source**，哨兵会踩
`Type support not from this implementation`）。

⭐ **开关改了要重跑 cmake**（`core/debug.hpp` 的实验开关、`CMakeLists.txt` 的
`HZMIR_WITH_ROS2`、`drivers/CMakeLists.txt` 的 `HZMIR_HAS_*` —— 都只在 **configure 阶段**被读）。
`build.sh` 会**自动检测并重跑**；手动 `cmake --build` 不会。
> 另有 `test_assert_live` 在运行期再次核对。

### 跑起来（零硬件）

```bash
./build/src/infantry --video=demo.avi --force-mode=1 --csv=run params/robots/infantry.yaml
#                                   ↑ 0=IDLE 1=自瞄 2=小符 3=大符
```

---

## ⭐⭐ 可选优化：**默认全关，一键开启**

| 优化 | 同济默认 | 本项目建议 | 如何开启 |
|---|---|---|---|
| 弹道实现 | `ideal`（无阻力斜抛） | `table`（离线 RK4 建表，**0.04 µs 查表**） | yaml 里 `trajectory_impl: table` |
| 检测器 | `yolo`（YOLOv5，**同济**） | `traditional`（传统灯条配对） | yaml 里 `detector_impl: traditional` |
| 弹道（更多档） | `ideal` | `rk4` / `rk4_drag` / `rk4_42` / `table_42` / `tongji_linear` | 同上，`trajectory_impl:` 换名 |
| 射击过滤 | 仅颜色 | + 编号 / 无敌 / 集火（5 条规则） | yaml 里 `armor_filter:` |
| 目标优先级 | 不设 | 4 张表（3/4 号优先等） | yaml 里 `priority_mode: 3` |
| EKF 一致性阈值 | `0.711` | χ²(4) 的 95% 分位 `9.4877` | `--tongji=false` |
| 小陀螺判据 | `ekf_x()[8]` | `ekf_x()[7]`（真正的角速度） | `--tongji=false` |
| 打符推理设备 | 硬编码 `CPU` | 可配 `GPU`/`AUTO` | yaml 里 `model_device:` |

```bash
# 完全同济行为（默认）
./src/infantry params/robots/infantry.yaml
# 启用本项目的优化
./src/infantry --tongji=false params/robots/infantry.yaml
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

`docs/` 下有 **5 份**分析与设计文档（约 1200 行），索引见 [`docs/README.md`](docs/README.md)：

| 文档 | 内容 |
|---|---|
| [`12-相对同济的变更审计.md`](docs/12-相对同济的变更审计.md) | ⭐ **每一处改动都可枚举**：27 个文件逐行 diff，13 个与上游逐字相同 |
| [`13-同济功能盘点与覆盖情况.md`](docs/13-同济功能盘点与覆盖情况.md) | 13 个分支**全部评估**，含 `poly` 的真相与 `auto_outpost` 的发现 |
| [`14-同济哨兵与本项目单相机哨兵对比.md`](docs/14-同济哨兵与本项目单相机哨兵对比.md) | `Decider` 10 个方法按「与相机数的关系」分类 |
| [`15-同济转换完整性报告.md`](docs/15-同济转换完整性报告.md) | ⭐ **135/135 源文件核对** · yaml 字段 53/59/63 全一致 · 13→4 主程序 |
| [`16-调试体系设计与可融入项.md`](docs/16-调试体系设计与可融入项.md) | ⭐ **双轨制原则** · 10 项必须保留的资产 · 可融入候选 · 全部待做清单 |

⭐ **每一处改动都能枚举、能关闭** —— 这是本项目对上有的最高承诺。

⭐ **每个目录都有自己的 `README.md` 写明定位**（加文件前先看一眼）：

| 目录 | README | 内容 |
|---|---|---|
| `params/` | [`params/README.md`](params/README.md) | ⭐ **配置两层**：`cameras/`(品牌) vs `robots/`(兵种) |
| `tools/` | [`tools/README.md`](tools/README.md) | 调试工具（`csv_viewer.html` / `diagnose_frame.py` / `split_video.py`） |
| `tools/scripts/` | [`tools/scripts/README.md`](tools/scripts/README.md) | 构建 / 运行 / 硬件脚本 |
| `scripts/` | [`scripts/README.md`](scripts/README.md) | 业务 Python（CSV 分析 / A-B 对比 / 画图） |
| `test/function/` | [`test/function/README_EXP.md`](test/function/README_EXP.md) | ⭐ **实验层规则**（三层测试策略） |

---

## 许可与致谢

⭐ **MIT License** —— 本项目是**同济大学 SuperPower 战队
[`sp_vision_25`](https://github.com/TongjiSuperPower/sp_vision_25)**（MIT）的衍生作品，
其版权声明已按 MIT 要求保留在 [LICENSE](LICENSE) 中。

完整归属、**不含哪些内容、为什么**，见 [THIRD_PARTY.md](THIRD_PARTY.md)。

特别感谢同济大学 SuperPower 战队开源了完整的自瞄/打符实现。
