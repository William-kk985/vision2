#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hzmir_csv.py —— ⭐ 读 `CsvSink` 输出的公共模块（`*_frames.csv` / `*_series.csv`）

⚠️ **零 pandas 依赖**（只用 stdlib `csv` + `numpy`）—— 赛场上不一定有 pandas。

用法：
    from hzmir_csv import load_frames, load_series, budget_table
    d = load_frames("run_frames.csv")
    d.col("pln_t_fly")        # -> np.ndarray
"""
from __future__ import annotations
import csv, os
from dataclasses import dataclass
from typing import Dict, List, Optional
import numpy as np

# ⭐ 性能列（自动识别"占帧预算"的项）
PERF_COLS = [
    # ⭐ W86：`t_cam_wait_us` = 阻塞等相机（**不耗 CPU**，等于帧率上限），
    #   拆出来后 `t_perceive_us` 才是真正的活。预算表要把 cam_wait 单独看待。
    "t_cam_wait_us", "t_perceive_us", "det_t_infer_us", "sol_t_solve_us", "trk_t_track_us",
    "tgt_t_update_us", "pln_t_plan_us", "ctl_t_us", "buff_t_us",
]
FRAME_BUDGET_US = 10000.0   # 10 ms（100 Hz）


@dataclass
class Frames:
    header: List[str]
    data: Dict[str, np.ndarray]

    def __len__(self) -> int:
        return len(next(iter(self.data.values()))) if self.data else 0

    def col(self, name: str) -> np.ndarray:
        if name not in self.data:
            raise KeyError(f"没有这一列: {name}\n可用列: {', '.join(self.header)}")
        return self.data[name]

    def has(self, name: str) -> bool:
        return name in self.data

    def tail(self, n: int) -> "Frames":
        """⭐ 取后 n 帧（用于剔热身帧）"""
        return Frames(self.header, {c: v[-n:] if n > 0 else v[:0] for c, v in self.data.items()})

    def nonzero_cols(self) -> List[str]:
        """⭐ 至少有一个非零值的列（合成数据里大量列恒 0，读起来很吵）"""
        return [c for c in self.header if np.any(self.data[c] != 0)]

    def const_cols(self) -> List[str]:
        return [c for c in self.header if np.all(self.data[c] == self.data[c][0])]


def load_frames(path: str) -> Frames:
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        header = next(r)
        rows = [row for row in r if row and len(row) == len(header)]
    if not rows:
        return Frames(header, {c: np.zeros(0) for c in header})
    arr = np.array(rows, dtype=float)
    return Frames(header, {c: arr[:, i] for i, c in enumerate(header)})


def load_series(path: str):
    """返回 {key: (t_us, value)}"""
    out: Dict[str, List] = {}
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            k = row["key"]
            out.setdefault(k, ([], []))
            out[k][0].append(float(row["t_us"]))
            out[k][1].append(float(row["value"]))
    return {k: (np.array(v[0]), np.array(v[1])) for k, v in out.items()}


# ⭐⭐ 已知的**嵌套关系**（父段包含子段）—— 求和时会**重复计算**，需排除子段
#
# ⚠️ W73 修正：原来这里写的是 `t_frame_us` 包含全部，但 `t_frame_us` **不在 `PERF_COLS` 里**
#    ⇒ 这个映射**完全没生效**，导致帧预算表把嵌套子段也加进合计（实测虚高到 111%）。
#
# ⭐ 真实的嵌套（W71/W72 补计时后实测发现）：
#   `Tracker::track()` 内部会调用 `solver_.solve(armor)`（PnP + yaw 优化）
#   和 `target_.update(armor)`（EKF）⇒ 这两个是 `trk_t_track_us` 的**子段**。
NESTED = {
    "trk_t_track_us": ["sol_t_solve_us", "tgt_t_update_us"],
}

# 展开成「子段集合」（求和时跳过它们，只留最外层）
NESTED_CHILDREN = frozenset(c for kids in NESTED.values() for c in kids)


def budget_table(d: Frames) -> List[tuple]:
    """⭐ 性能列的「帧预算占用」表：(列名, 均值µs, 最大µs, 占比%)

    ⚠️ 各段**不一定正交** —— `NESTED` 里列的子段是父段的一部分，
       它们**不计入合计**（否则合计虚高；曾实测到 111%）。
       返回的元组第 5 位 `is_child` 用于显示时加标注。
    """
    rows = []
    for c in PERF_COLS:
        if not d.has(c):
            continue
        v = d.col(c)
        if not np.any(v):          # 全 0 的段（该兵种没走这条路径）
            continue
        rows.append((
            c, float(v.mean()), float(v.max()), float(v.mean()) / FRAME_BUDGET_US * 100,
            c in NESTED_CHILDREN,
        ))
    rows.sort(key=lambda x: -x[1])
    return rows


def stats(v: np.ndarray) -> dict:
    v = v[np.isfinite(v)]
    if v.size == 0:
        return dict(n=0)
    return dict(
        n=int(v.size), mean=float(v.mean()), std=float(v.std()),
        min=float(v.min()), p50=float(np.percentile(v, 50)),
        p95=float(np.percentile(v, 95)), max=float(v.max()),
    )


def setup_matplotlib():
    """⭐ matplotlib 需要一个可写的 config 目录（默认 ~/.config 可能不可写）"""
    os.environ.setdefault("MPLCONFIGDIR", os.path.join(os.getcwd(), ".mplconfig"))
    os.makedirs(os.environ["MPLCONFIGDIR"], exist_ok=True)
    import matplotlib
    matplotlib.use("Agg")     # 无显示环境也能存图
    import matplotlib.pyplot as plt
    return plt
