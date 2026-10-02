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



def detect_diagnosis(d) -> None:
    """⭐⭐⭐ W65：用 `det_nms` / `det_armor_count` / `det_best_conf` 三列**组合诊断检测链路**

    ## 为什么需要
    `det_armor_count` 是【过滤后】的数量（`check_name`/`check_type` 在 `detect()` 内就删了），
    所以它 = 0 时**分不清**：
      · YOLO **根本没候选**（objectness 没过）        ← 没检出
      · YOLO **有候选但被滤掉**（not_armor/置信度/类型）← 检出了但不用
    ⭐ 用 `det_nms`（NMS 存活数，过滤前）就能区分开。

    | det_nms | det_armor_count | 含义 |
    |---|---|---|
    | 0 | 0 | ⭐ **YOLO 没候选**（objectness 没过） |
    | >0 | 0 | ⭐ **检出了但被滤掉**（not_armor / 置信度 / 类型） |
    | >0 | >0 | ✅ 正常输出 |
    """
    need = ("det_nms", "det_armor_count", "det_best_conf")
    if not all(c in d.header for c in need):
        return
    nms = d.col("det_nms").astype(np.int64)
    cnt = d.col("det_armor_count").astype(np.int64)
    conf = d.col("det_best_conf").astype(np.float64)
    n = len(nms)
    if n == 0:
        return

    no_cand = int(np.sum((nms == 0) & (cnt == 0)))          # YOLO 没候选
    filtered = int(np.sum((nms > 0) & (cnt == 0)))          # 有候选但全被滤掉
    ok = int(np.sum(cnt > 0))                                # 正常输出

    print("\n── ⭐ 检测链路诊断（det_nms → det_armor_count）──")
    print(f"  {'✅ 正常输出':<22} {ok:>6} 帧  ({ok / n * 100:5.1f}%)")
    print(f"  {'⭐ 有候选但全被滤掉':<20} {filtered:>6} 帧  ({filtered / n * 100:5.1f}%)"
          f"   ← check_name / check_type 删掉的")
    print(f"  {'⚠️ YOLO 没候选':<22} {no_cand:>6} 帧  ({no_cand / n * 100:5.1f}%)"
          f"   ← objectness 没过，YOLO 没看到装甲板")

    # ── 置信度 ──
    if ok > 0:
        c_ok = conf[cnt > 0]
        print(f"\n  输出帧的置信度: 均值 {c_ok.mean():.3f}  最低 {c_ok.min():.3f}  "
              f"最高 {c_ok.max():.3f}")

    # ── ⚠️ 列没填的检测（历史 bug：best_conf/nms 从没被赋值）──
    if ok > 0 and not np.any(conf > 0):
        print("  ⚠️⚠️ **有检出但 det_best_conf 全是 0** → 这一列**没被赋值**（历史 bug）")
    if ok > 0 and not np.any(nms > 0):
        print("  ⚠️⚠️ **有检出但 det_nms 全是 0** → 这一列**没被赋值**（历史 bug）")

    # ── 结论 ──
    print("  ── 判读 ──")
    if filtered > n * 0.2:
        print(f"  · ⭐ **{filtered / n * 100:.0f}% 的帧「YOLO 看到了但被滤掉」** → "
              f"查 `check_name`(not_armor/置信度) 与 `check_type`(大小装甲板不符)")
        print("    → 跑程序时加 `--det-stats` 看终端会打印各步各滤掉几个")
    elif no_cand > n * 0.5:
        print(f"  · ⚠️ **{no_cand / n * 100:.0f}% 的帧 YOLO 完全没候选** → 多是画面里没有装甲板，")
        print("    或装甲板**数字缺失/模糊**（实测：遮住数字 → objectness 直接不过阈值）")
    elif ok > n * 0.5:
        print("  · ✅ 检测链路**正常**（多数帧有输出）")
    else:
        print("  · 🔶 三种情况都有，建议结合终端 `--det-stats` 日志看具体某帧")


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

    # ── ⭐⭐ 全 0 列警告：默认会**隐藏**它们，但"永远为 0"往往是【忘了填】而非【真为 0】 ──
    zero_cols = [c for c in d.header if not np.any(d.col(c))]
    if zero_cols and not a.all:
        print(f"\n╔═ ⚠️ **{len(zero_cols)} 个列【全为 0】，已被隐藏** ═══════════════════════════")
        print(f"║  请确认是【真为 0】还是【**忘了填**】（后者是静默 bug，看数据看不出来）")
        for i in range(0, min(len(zero_cols), 24), 4):
            print("║    " + "  ".join(f"{c:<20}" for c in zero_cols[i:i+4]))
        if len(zero_cols) > 24:
            print(f"║    ...（还有 {len(zero_cols)-24} 个，用 `--all` 看全部）")
        print(f"╚═ 提示：查代码里该字段的赋值点；本项目已知的：`game_state`（TODO 没接比赛状态）")

    # ── ⭐⭐⭐ 检测链路诊断（三列组合）──
    detect_diagnosis(d)

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
