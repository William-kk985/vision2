# 效率对比 harness

对比**本项目**与**同济上游**（`vision_v2/upstream/sp_vision_25`）的纯算法耗时。
产出见 [docs/20-运行效率对比-改造架构vs同济.md](../../docs/20-运行效率对比-改造架构vs同济.md)。

## 为什么需要专门的 harness

⚠️ 上游自带的 `tests/detector_video_test` **不能直接用来计时**：
```cpp
auto key = cv::waitKey(33);   // 每帧固定睡 33 ms ⇒ 687 帧 = 22.7 s
plotter.plot(data);           // 另接 PlotJuggler
```
⇒ 其墙钟时间（曾测得 29.2 s）**主要来自硬睡**，不代表算法耗时。

## ⚠️⚠️ 测量纪律：必须【交替执行】

实测同一份 `detect()` 在不同机器状态下：

| 机器状态 | `det` 中位 |
|---|---|
| 刚冷却 | **5825 μs** |
| 连续跑数轮基准后 | **7780 ~ 8005 μs** |

⇒ **差 35%**，且可复现（YOLO 推理是 AVX 密集负载，连续满负载触发 CPU 降频）。
用 `probe_throttle` 可复现该现象：

```
sleep=0us      median=7893us
sleep=10000us  median=6743us    ← 降 14.6%
sleep=0us      median=7876us    ← 回到 7876，可复现
```

⭐ **因此：两边必须逐轮交替执行**（TJ → OURS → TJ → OURS …），使二者经历相同热状态。
⚠️ 非交替测量曾得出 **"+42%"** 与 **"−5%"** 两种**相反**结论，均已作废。

## 构建与使用

```bash
cd tools/bench
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# 本项目（可在任意目录运行）
./build/bench_ours <本项目 params/robots/X.yaml> <录像> [nostats]

# 上游（⚠️ 必须 cd 到上游目录，因为它用相对路径找 assets/）
cd ../../../upstream/sp_vision_25
<本项目>/tools/bench/build/bench_tongji configs/demo.yaml <录像>

# 两个探针
./build/probe_img      <本项目 config> <录像>    # 两条取图路径的图是否一致
./build/probe_throttle <本项目 config> <录像>    # 验证 CPU 热降频
```

⚠️ 本项目静态库路径默认取 `exp/hzmir_build`，可用 `-DHB=<构建目录>` 覆盖。

## 结论摘要

| 阶段 | 同济 | 本项目 | 差异 |
|---|---|---|---|
| `det` | 5822 μs | 5967 μs | +2.5% |
| `track` | 281 μs | 310 μs | +10.1%（绝对 +29 μs） |
| `plan` | 232 μs | 242 μs | +4.5% |
| **合计** | **6334 μs** | **6519 μs** | **+2.9%** = +0.19 ms/帧 |

⭐ 占 30 fps 预算（33 ms）的 **0.6%**。
