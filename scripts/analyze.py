#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze.py —— ⭐ 读 `*_frames.csv` 出统计摘要（doc 09 §14.13 第⑥层）

用法：
    python3 scripts/analyze.py run_frames.csv [--all] [--perf] [--group mode]
    python3 scripts/analyze.py run_frames.csv --budget      # 只看帧预算占用
"""
import argparse, sys
sys.path.insert(0, __import__("os").path.dirname(__file__))
import numpy as np
from hzmir_csv import load_frames, budget_table, stats, FRAME_BUDGET_US


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames_csv")
    ap.add_argument("--all", action="store_true", help="列出全部列（含恒 0 的）")
    ap.add_argument("--budget", action="store_true", help="只打印帧预算占用表")
    ap.add_argument("--group", default=None, help="按某列分组统计（如 mode）")
    ap.add_argument("--warmup", type=int, default=0,
                    help="⭐ 剔除前 N 帧（推理热身帧非常慢，会拉高均值）")
    a = ap.parse_args()

    d = load_frames(a.frames_csv)
    if a.warmup > 0:
        print(f"  ⭐ 剔除前 {a.warmup} 帧热身（原 {len(d)} 帧）")
        d = d.tail(len(d) - a.warmup)
    print(f"═══ {a.frames_csv}")
    print(f"  {len(d)} 帧 · {len(d.header)} 列")

    # ── 帧预算 ──
    bt = budget_table(d)
    if bt:
        print("\n── 帧预算占用（10 ms 周期 @100 Hz）──")
        print(f"  {'段':<20} {'均值':>10} {'最大':>10} {'占均值':>9}")
        for c, mean, mx, pct in bt:
            print(f"  {c:<20} {mean:>8.1f}us {mx:>8.1f}us {pct:>7.2f}%")
        total = sum(r[1] for r in bt)
        print(f"  {'合计':<20} {total:>8.1f}us {'':>10} {total/FRAME_BUDGET_US*100:>7.2f}%")
    if a.budget:
        return

    # ── 分组 ──
    if a.group:
        if not d.has(a.group):
            print(f"\n⚠️ 没有分组列 '{a.group}'"); return
        g = d.col(a.group)
        print(f"\n── 按 {a.group} 分组 ──")
        for val in np.unique(g):
            print(f"  {a.group}={int(val)}: {int((g == val).sum())} 帧"
                  f" ({100*(g == val).mean():.1f}%)")
        return

    # ── 逐列统计 ──
    cols = d.header if a.all else d.nonzero_cols()
    print(f"\n── 逐列统计（{'全部' if a.all else '非全零'} {len(cols)} 列）──")
    print(f"  {'列':<22} {'n':>6} {'均值':>12} {'p50':>12} {'p95':>12} {'最大':>12}")
    for c in cols:
        s = stats(d.col(c))
        if s.get("n", 0) == 0:
            continue
        print(f"  {c:<22} {s['n']:>6} {s['mean']:>12.4f} {s['p50']:>12.4f} "
              f"{s['p95']:>12.4f} {s['max']:>12.4f}")

    const = d.const_cols()
    if const and not a.all:
        print(f"\n  （{len(const)} 列恒定: {', '.join(const[:8])}{' ...' if len(const) > 8 else ''}）")


if __name__ == "__main__":
    main()
