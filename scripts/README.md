# `scripts/` —— Python 快速验证 / 离线算法可视化

> ⭐ 这是**六层调试体系的第 ⑥ 层**（doc 09 §14.13）：
> `FrameDebug` → `CsvSink` → **`scripts/*.py`（Pandas/numpy）→ 画图 / 统计 / 对比 A-B**

**设计原则**
- ⚠️ **零 pandas 依赖**（只用 stdlib `csv` + `numpy`/`matplotlib`）—— 赛场上不一定装 pandas
- 自动**跳过全零列**（合成数据里大量列恒 0，读起来很吵）
- ⭐ `analyze.py` 自动识别**性能列**并算「帧预算占用」

---

## 1. 怎么产生数据

```bash
# 录像回放 + 落 CSV（主链路一行不用改）
./src/infantry --video=demo.avi --force-mode=1 --csv=run params/robots/infantry.yaml
#                                          ↑ 0=IDLE 1=自瞄 2=小符 3=大符
# 产出 run_frames.csv（57 列）+ run_series.csv（曲线）
```

---

## 2. `analyze.py` —— 统计摘要

```bash
python3 scripts/analyze.py output/csv/run_frames.csv                 # 预算表 + 非全零列统计
python3 scripts/analyze.py output/csv/run_frames.csv --budget         # 只看帧预算占用
python3 scripts/analyze.py output/csv/run_frames.csv --warmup 3       # ⭐ 剔除前 3 帧（推理热身）
python3 scripts/analyze.py output/csv/run_frames.csv --all            # 含恒 0 的列
python3 scripts/analyze.py output/csv/run_frames.csv --group mode     # 按档位分组
```

**实测输出**（CPU 推理，自瞄档，40 帧）：
```
── 帧预算占用（10 ms 周期 @100 Hz）──
  段                            均值         最大       占均值
  det_t_infer_us         7266.6us  47862.0us   72.67%     ← YOLO 推理
  t_cam_wait_us          3190.0us   4500.0us   31.90%     ← 等相机/解码（**阻塞，不耗 CPU**）
  t_perceive_us             6.2us     50.0us    0.06%     ← 真正的取图后处理
  trk_t_track_us            1.6us      2.0us    0.02%     ← 跟踪几乎不花时间
  合计                     9335.1us              93.35%

剔除前 3 帧（热身）后：
  det_t_infer_us         6263.9us   7628.0us   62.64%
  合计                     8274.1us              82.74%
```

⭐ **首帧 47.9 ms vs 稳态 6.3 ms（7.6 倍）** —— 那是 OpenVINO 的**首次推理热身**。

---

## 3. `plot.py` —— ⭐ 离线算法可视化

```bash
python3 scripts/plot.py run_frames.csv --preset plan      # 规划器内部量
python3 scripts/plot.py run_frames.csv --preset perf --hist
python3 scripts/plot.py run_frames.csv --cols pln_t_fly,pln_overlap -o x.png
python3 scripts/plot.py run_series.csv --series           # 画 series.csv
```

**预设**（把「一起看才有意义」的量分好组）：

| preset | 内容 |
|---|---|
| `plan` | `pln_t_fly` / `pln_overlap` / `pln_acc_max` / `pln_iters` / `sht_traj_err` |
| `target` | `tgt_x/y/z` / `tgt_yaw` / `tgt_w` / `tgt_nis` |
| `track` | `trk_state` / `trk_armor_count` / `trk_filtered_out` / `trk_priority_mode` |
| `perf` | 全部 `*_us` 性能列 |
| `detector` / `buff` / `control` | 对应角色的量 |

⭐ 无显示环境也能用（内部走 `Agg` 后端，直接存 PNG）。
⭐ 会自动跳过**全为 0 的列**（该路径本帧没走），不会画出空白图。

---

## 4. `compare.py` —— ⭐⭐⭐ A/B 实验对比

**这是「用数据决策」的核心工具**：改一个参数 → 各跑一遍 → 对比两份 CSV。

```bash
python3 scripts/compare.py A_frames.csv B_frames.csv --label-a 同济 --label-b 优化
python3 scripts/compare.py A.csv B.csv --cols pln_t_fly,pln_acc_max --min-rel 0.05
```

**实测输出**（自瞄 vs 打符）：
```
── 帧预算（均值 µs）──
  段                              自瞄           打符         变化
  t_perceive_us               6.2us       50.0us      +1.6%

── 逐列对比（按变化幅度排序）──
  buff_t_us                      0        13483   新出现 🔴
  det_t_infer_us            7266.6            0  -100.0% 🟢
  t_frame_us                9340.5        15584   +66.8% 🔴   ← 打符单帧更贵
  ⭐ 合计纳入比较 8 列，显著差异 6 列
```

⭐ 若**没有任何列超过阈值** → 会明确打印
`（没有超过阈值的差异 → 这次改动**没有可观测效果**）`
—— **避免"改了个寂寞还以为有效果"**。

---

## 5. `hzmir_csv.py` —— 公共模块

```python
from hzmir_csv import load_frames, load_series, budget_table, stats, setup_matplotlib
d = load_frames("run_frames.csv")
d.col("pln_t_fly")        # -> np.ndarray
d.nonzero_cols()          # 至少有一个非零值的列
d.tail(10)                # 后 10 帧（剔热身）
budget_table(d)           # [(列名, 均值µs, 最大µs, 占比%)]
```

**写自己的分析脚本时用它**，不用重复写 CSV 解析。
