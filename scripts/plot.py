#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot.py —— ⭐ 离线算法可视化（doc 09 §14.13 第⑥层）

**这就是「算法级调试」**：不改代码、不接硬件，直接从 CSV 画内部量曲线。

用法：
    python3 scripts/plot.py run_frames.csv --preset plan
    python3 scripts/plot.py run_frames.csv --cols pln_t_fly,pln_overlap,pln_acc_max -o out.png
    python3 scripts/plot.py run_series.csv --series          # 画 series.csv 的曲线
    python3 scripts/plot.py run_frames.csv --preset perf
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np
from hzmir_csv import load_frames, load_series, setup_matplotlib, PERF_COLS

# ⭐ 预设：把「一起看才有意义」的量分好组
PRESETS = {
    "plan":      ["pln_t_fly", "pln_overlap", "pln_acc_max", "pln_iters", "sht_traj_err"],
    "target":    ["tgt_x", "tgt_y", "tgt_z", "tgt_yaw", "tgt_w", "tgt_nis"],
    "track":     ["trk_state", "trk_armor_count", "trk_filtered_out", "trk_priority_mode"],
    "perf":      PERF_COLS,
    "detector":  ["det_armor_count", "det_best_conf", "det_nms"],
    "buff":      ["buff_fanblade_count", "buff_spd", "buff_solved", "buff_t_us"],
    "control":   ["ctl_yaw", "ctl_pitch", "ctl_control", "ctl_shoot"],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--preset", choices=list(PRESETS), default=None)
    ap.add_argument("--cols", default=None, help="逗号分隔的列名")
    ap.add_argument("--out", default=None, help="输出 png（默认 <csv>_<preset>.png）")
    ap.add_argument("--series", action="store_true", help="输入是 series.csv")
    ap.add_argument("--hist", action="store_true", help="同时画直方图")
    a = ap.parse_args()

    plt = setup_matplotlib()

    if a.series:
        data = load_series(a.csv)
        keys = sorted(data)
        if not keys:
            print("series.csv 里没有曲线"); return
        fig, axes = plt.subplots(len(keys), 1, figsize=(11, 2.2 * len(keys)), squeeze=False)
        for ax, k in zip(axes[:, 0], keys):
            t, v = data[k]
            ax.plot(t, v, lw=0.9); ax.set_ylabel(k, fontsize=8)
            ax.grid(alpha=.3)
        axes[-1, 0].set_xlabel("frame_id")
        fig.suptitle(f"series: {os.path.basename(a.csv)}", fontsize=10)
        fig.tight_layout()
        out = a.out or os.path.splitext(a.csv)[0] + "_series.png"
        fig.savefig(out, dpi=110); print(f"✅ {out}")
        return

    d = load_frames(a.csv)
    if a.cols:
        cols = [c.strip() for c in a.cols.split(",")]
    elif a.preset:
        cols = PRESETS[a.preset]
    else:
        print("请给 --preset 或 --cols；可用 preset:", ", ".join(PRESETS)); return

    cols = [c for c in cols if d.has(c) and np.any(d.col(c) != 0)]
    if not cols:
        print(f"⚠️ 选中的列在 {a.csv} 里全为 0 或不存在（该路径本帧没走）"); return

    n = len(cols)
    fig, axes = plt.subplots(n, 1, figsize=(11, 1.9 * n), squeeze=False, sharex=True)
    x = d.col("frame_id") if d.has("frame_id") else np.arange(len(d))
    for ax, c in zip(axes[:, 0], cols):
        v = d.col(c)
        ax.plot(x, v, lw=0.9, color="tab:blue")
        ax.set_ylabel(c, fontsize=8); ax.grid(alpha=.3)
        s = v[np.isfinite(v)]
        if s.size:
            ax.axhline(s.mean(), color="tab:red", ls="--", lw=0.7, alpha=.7)
            ax.text(.99, .05, f"mean={s.mean():.4g}  max={s.max():.4g}",
                    transform=ax.transAxes, ha="right", fontsize=7, color="tab:red")
    axes[-1, 0].set_xlabel("frame_id")
    fig.suptitle(f"{os.path.basename(a.csv)}  ({len(d)} frames)", fontsize=10)
    fig.tight_layout()
    out = a.out or os.path.splitext(a.csv)[0] + f"_{a.preset or 'sel'}.png"
    fig.savefig(out, dpi=110); print(f"✅ {out}")

    if a.hist:
        fig2, axes2 = plt.subplots(1, n, figsize=(3.4 * n, 2.6), squeeze=False)
        for ax, c in zip(axes2[0], cols):
            ax.hist(d.col(c), bins=40, color="tab:blue", alpha=.8)
            ax.set_title(c, fontsize=8); ax.grid(alpha=.3)
        fig2.tight_layout()
        out2 = os.path.splitext(out)[0] + "_hist.png"
        fig2.savefig(out2, dpi=110); print(f"✅ {out2}")


if __name__ == "__main__":
    main()
