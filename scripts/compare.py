#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare.py —— ⭐⭐⭐ A/B 实验对比（doc 09 §14.13「对比 A-B」）

**这是「用数据决策」的核心工具**：
改一个参数 → 各跑一遍 → 对比两份 CSV → 看差异是否显著。

用法：
    python3 scripts/compare.py A_frames.csv B_frames.csv
    python3 scripts/compare.py A.csv B.csv --label-a 同济 --label-b 优化
    python3 scripts/compare.py A.csv B.csv --cols pln_t_fly,pln_acc_max --min-rel 0.05
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np
from hzmir_csv import load_frames, budget_table, stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a"); ap.add_argument("b")
    ap.add_argument("--label-a", default="A"); ap.add_argument("--label-b", default="B")
    ap.add_argument("--cols", default=None, help="只比这些列（默认自动）")
    ap.add_argument("--min-rel", type=float, default=0.02,
                    help="相对变化小于此值视为'无变化'（默认 2%%）")
    a_ = ap.parse_args()

    A, B = load_frames(a_.a), load_frames(a_.b)
    print(f"  A = {a_.label_a}: {len(A)} 帧  ({a_.a})")
    print(f"  B = {a_.label_b}: {len(B)} 帧  ({a_.b})")

    # ── 帧预算对比 ──
    ba, bb = dict((r[0], r) for r in budget_table(A)), dict((r[0], r) for r in budget_table(B))
    keys = [k for k in ba if k in bb]
    if keys:
        print(f"\n── 帧预算（均值 µs）──")
        print(f"  {'段':<20} {a_.label_a:>12} {a_.label_b:>12} {'变化':>10}")
        for k in sorted(keys, key=lambda x: -ba[x][1]):
            va, vb = ba[k][1], bb[k][1]
            rel = (vb - va) / va * 100 if va else float("inf")
            flag = "🔴" if rel > 10 else ("🟢" if rel < -10 else "  ")
            print(f"  {k:<20} {va:>10.1f}us {vb:>10.1f}us {rel:>+9.1f}% {flag}")

    # ── 逐列对比 ──
    cols = ([c.strip() for c in a_.cols.split(",")] if a_.cols
            else [c for c in A.header if c in B.header])
    rows = []
    for c in cols:
        sa, sb = stats(A.col(c)), stats(B.col(c))
        if sa.get("n", 0) == 0 or sb.get("n", 0) == 0:
            continue
        ma, mb = sa["mean"], sb["mean"]
        if abs(ma) < 1e-12 and abs(mb) < 1e-12:
            continue                                  # 两边都全 0
        rel = (mb - ma) / abs(ma) if abs(ma) > 1e-12 else float("inf")
        rows.append((c, ma, mb, rel, sa, sb))

    rows.sort(key=lambda r: -abs(r[3]) if np.isfinite(r[3]) else -1e9)
    print(f"\n── 逐列对比（按变化幅度排序；|变化| < {a_.min_rel*100:.0f}% 视为无变化）──")
    print(f"  {'列':<22} {a_.label_a+' 均值':>14} {a_.label_b+' 均值':>14} {'变化':>10}")
    shown = 0
    for c, ma, mb, rel, sa, sb in rows:
        if np.isfinite(rel) and abs(rel) < a_.min_rel:
            continue
        flag = "🔴" if rel > 0 else "🟢"
        rtxt = "  新出现" if not np.isfinite(rel) else f"{rel*100:>+9.1f}%"
        print(f"  {c:<22} {ma:>14.5g} {mb:>14.5g} {rtxt} {flag}")
        shown += 1
    if shown == 0:
        print("  （没有超过阈值的差异 → 这次改动**没有可观测效果**）")
    print(f"\n  ⭐ 合计纳入比较 {len(rows)} 列，显著差异 {shown} 列")


if __name__ == "__main__":
    main()
