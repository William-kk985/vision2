#!/usr/bin/env bash
# ⭐⭐⭐ 录制-回放离线调参流程（W100 · 原 C1）
#
# ## 为什么需要
# 调算法的正确姿势是「**录一次，反复回放**」—— 而不是每次改参数都上真机跑一次。
# 组件其实都有（`Recorder` 录 `.avi`+`.txt`、`ReplayCBoard` 回放位姿、
# `VideoCamera` 回放画面），⚠️ **但缺一条好用的流程**：
#   · 录制文件名是**时间戳**（`2026-10-03_11-30-00.avi`）→ 回头找不着
#   · 回放/分析/对比要手敲一长串参数
#
# ## 用法
#   tools/scripts/tune.sh record  infantry --name=test1 --secs=20
#   tools/scripts/tune.sh list
#   tools/scripts/tune.sh replay  test1 --tongji=false          # ⭐ 反复调参
#   tools/scripts/tune.sh analyze test1
#   tools/scripts/tune.sh compare base test1
#   tools/scripts/tune.sh clean --days=7
#
# ⭐ 核心价值：**录一次 ↔ 回放无数次**，每次改参数只要 `replay`。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VID="$ROOT/output/video"
CSV="$ROOT/output/csv"
mkdir -p "$VID" "$CSV"

c_red=$'\033[31m'; c_grn=$'\033[32m'; c_ylw=$'\033[33m'; c_dim=$'\033[2m'; c_rst=$'\033[0m'
die() { echo "${c_red}✗ $*${c_rst}" >&2; exit 1; }
ok()  { echo "${c_grn}✓${c_rst} $*"; }
dim() { echo "${c_dim}$*${c_rst}"; }

# ⭐ 把 `<时间戳>.avi` 重命名成 `<name>.avi`（连同同名 .txt）
rename_latest() {
  local name="$1"
  local latest
  latest="$(ls -t "$VID"/*.avi 2>/dev/null | head -1 || true)"
  [[ -n "$latest" ]] || die "没找到刚录的文件（$VID/*.avi）"
  local base="${latest%.avi}"
  [[ "$(basename "$base")" == "$name" ]] && { ok "已是 $name.avi"; return; }
  mv -f "$latest" "$VID/$name.avi"
  [[ -f "$base.txt" ]] && mv -f "$base.txt" "$VID/$name.txt"
  ok "重命名 → ${c_ylw}$name.avi${c_rst}（+ $name.txt）"
}

# ⭐ 找一段录制；支持 `latest` 和模糊匹配
resolve() {
  local q="$1"
  if [[ "$q" == "latest" ]]; then
    ls -t "$VID"/*.avi 2>/dev/null | head -1 | sed 's/\.avi$//'
    return
  fi
  local hit
  hit="$(ls -t "$VID"/*"$q"*.avi 2>/dev/null | head -1 | sed 's/\.avi$//' || true)"
  [[ -n "$hit" ]] || die "找不到匹配 '$q' 的录制（试试 tune.sh list）"
  echo "$hit"
}

cmd_record() {
  local robot="${1:?用法: tune.sh record <robot> [--name=X] [--secs=N] [其它]]}"
  shift || true
  local name="" secs=""
  local extra=()
  for a in "$@"; do
    case "$a" in
      --name=*) name="${a#--name=}" ;;
      --secs=*) secs="${a#--secs=}" ;;
      *) extra+=("$a") ;;
    esac
  done
  [[ -n "$name" ]] || die "必须给 --name=<好记的名字>（否则回头找不着时间戳文件）"

  echo "${c_ylw}══ 录制 $robot → $name ══${c_rst}"
  dim "  ⚠️ 录制会占一个核做 MJPG 编码（实测对自瞄 +2.4%）"
  dim "  ⚠️ 需要真机（相机/下位机）；只跑录像回放不需要录"

  # ⭐ `--record` 开录；`--secs` 用 timeout 控制时长
  local tmo=()
  [[ -n "$secs" ]] && tmo=(timeout "$secs")
  set +e
  "${tmo[@]}" "$ROOT/tools/scripts/run.sh" "$robot" --record "${extra[@]+"${extra[@]}"}"
  local rc=$?
  set -e
  [[ $rc -eq 124 ]] && dim "  （$secs 秒到，已自动停）"

  rename_latest "$name"
  echo
  ok "录制完成。回放：${c_ylw}tools/scripts/tune.sh replay $name${c_rst}"
}

cmd_list() {
  echo "${c_ylw}══ 已录的段（$VID）══${c_rst}"
  [[ -d "$VID" ]] || { dim "  （空）"; return; }
  local found=0
  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    found=1
    local base="${f%.avi}" sz pose
    sz="$(du -h "$f" 2>/dev/null | cut -f1)"
    pose="—"; [[ -f "$base.txt" ]] && pose="$(wc -l < "$base.txt") 行位姿"
    printf "  %-34s %6s  %s\n" "$(basename "$base")" "$sz" "$pose"
  done < <(ls -t "$VID"/*.avi 2>/dev/null || true)
  [[ $found -eq 0 ]] && dim "  （空 —— 先跑 tune.sh record <robot> --name=X）"
}

cmd_replay() {
  local q="${1:?用法: tune.sh replay <名字|latest> [调参参数...]}"
  shift || true
  local base; base="$(resolve "$q")"
  local robot="${ROBOT:-infantry}"     # ⭐ 用 ROBOT=xxx 覆盖
  [[ -n "${2:-}" ]] && true
  echo "${c_ylw}══ 回放 $(basename "$base") ══${c_rst}"
  dim "  ⭐ 改参数后直接重跑这条命令即可（不用重新录）"
  dim "  想换兵种：ROBOT=hero tools/scripts/tune.sh replay $q ..."
  "$ROOT/tools/scripts/run.sh" "$robot" --video="$base.avi" --force-mode=1 \
    ${CSV_PREFIX+--csv="$CSV_PREFIX"} "$@"
}

cmd_analyze() {
  local q="${1:?用法: tune.sh analyze <名字|latest>}"
  local base; base="$(resolve "$q")"
  local pref="${CSV_PREFIX:-$CSV/$(basename "$base")}"
  echo "${c_ylw}══ 分析 $(basename "$base") ══${c_rst}"
  if [[ -f "${pref}_frames.csv" ]]; then
    ok "找到 CSV：${pref}_frames.csv"
  else
    dim "  没有现成 CSV → 先回放一次并落 CSV"
    CSV_PREFIX="$pref" cmd_replay "$q" --csv="$pref" --video-speed=0 || true
  fi
  if [[ -f "$ROOT/scripts/analyze.py" ]]; then
    python3 "$ROOT/scripts/analyze.py" "$pref" 2>/dev/null || \
      dim "  （analyze.py 跑失败，可手动看 ${pref}_frames.csv）"
  else
    dim "  ⭐ 没有 analyze.py —— 可用 tools/csv_viewer.html 打开 CSV（浏览器拖进去）"
  fi
}

cmd_compare() {
  local a="${1:?用法: tune.sh compare <A> <B>}"
  local b="${2:?用法: tune.sh compare <A> <B>}"
  local pa="$CSV/${a}" pb="$CSV/${b}"
  # ⭐ 允许传录制名 → 自动映射到同名 CSV 前缀
  [[ -f "${pa}_frames.csv" ]] || pa="$CSV/$(basename "$(resolve "$a")")"
  [[ -f "${pb}_frames.csv" ]] || pb="$CSV/$(basename "$(resolve "$b")")"
  [[ -f "${pa}_frames.csv" ]] || die "找不到 ${pa}_frames.csv"
  [[ -f "${pb}_frames.csv" ]] || die "找不到 ${pb}_frames.csv"
  echo "${c_ylw}══ 对比 $a  vs  $b ══${c_rst}"
  python3 - "$pa" "$pb" "$a" "$b" <<'PY'
import csv, io, sys, statistics as st
pa, pb, na, nb = sys.argv[1:5]
def load(p):
    r = list(csv.reader(io.open(p + "_frames.csv", encoding="utf-8")))
    return r[0], r[1:]
KEY = ["det_armor_count", "det_best_conf", "trk_armor_count", "trk_state",
       "tgt_x", "sht_should_fire", "t_frame_us", "t_cam_wait_us", "pln_t_fly"]
ha, da = load(pa); hb, db = load(pb)
print(f"  {'指标':<20}{na:>14}{nb:>14}{'变化':>14}")
print("  " + "-" * 62)
for c in KEY:
    if c not in ha or c not in hb: continue
    ia, ib = ha.index(c), hb.index(c)
    va = [float(x[ia]) for x in da if x[ia] not in ("",)]
    vb = [float(x[ib]) for x in db if x[ib] not in ("",)]
    if not va or not vb: continue
    # ⭐ 计数型看非零比例，数值型看均值
    cnt = c.endswith("_count") or c in ("trk_state", "sht_should_fire", "det_armor_count")
    if cnt:
        fa = sum(1 for x in va if x != 0) / len(va) * 100
        fb = sum(1 for x in vb if x != 0) / len(vb) * 100
        print(f"  {c:<20}{fa:>13.1f}%{fb:>13.1f}%{fb-fa:>+13.1f}pp")
    else:
        ma, mb = st.mean(va), st.mean(vb)
        d = (mb - ma) / ma * 100 if ma else 0
        print(f"  {c:<20}{ma:>14.3f}{mb:>14.3f}{d:>+13.1f}%")
print()
print("  ⭐ 计数型显示「非零帧占比」，数值型显示均值")
PY
}

cmd_clean() {
  local days=7
  for a in "$@"; do case "$a" in --days=*) days="${a#--days=}";; esac; done
  echo "${c_ylw}══ 清理 $days 天前的录制（$VID）══${c_rst}"
  local n
  n="$(find "$VID" -name '*.avi' -mtime "+$days" 2>/dev/null | wc -l)"
  if [[ "$n" -eq 0 ]]; then ok "没有超过 $days 天的录制"; return; fi
  dim "  将删除 $n 组（.avi + 同名 .txt）"
  find "$VID" -name '*.avi' -mtime "+$days" -print0 2>/dev/null | while IFS= read -r -d '' f; do
    rm -f "$f" "${f%.avi}.txt"
  done
  ok "已删 $n 组"
  dim "  ⚠️ 注意：这与会话整组删除（.avi + .txt 一起），不会留孤儿 .txt"
}

usage() {
  cat <<EOF
${c_ylw}tune.sh${c_rst} —— 录制-回放离线调参流程

  ${c_grn}record${c_rst}  <robot> --name=X [--secs=N] [参数...]   录一段（需要真机）
  ${c_grn}list${c_rst}                                            列出已录的段
  ${c_grn}replay${c_rst}  <名字|latest> [参数...]                 回放（⭐ 反复调参用这个）
  ${c_grn}analyze${c_rst} <名字|latest>                           分析（找/生成 CSV 并出报告）
  ${c_grn}compare${c_rst} <A> <B>                                 对比两段的指标
  ${c_grn}clean${c_rst}   [--days=7]                              清理旧录制

环境变量：
  ROBOT=hero        replay 时换兵种（默认 infantry）
  CSV_PREFIX=xxx    replay/analyze 的 CSV 前缀

⭐ 典型流程：
    # ① 上真机录一段（20 秒）
    tools/scripts/tune.sh record infantry --name=spin_test --secs=20
    # ② 之后【反复】回放调参，不用再上真机
    tools/scripts/tune.sh replay spin_test --tongji=false
    tools/scripts/tune.sh replay spin_test --nis-thresh=chi2
    # ③ 对比
    CSV_PREFIX=output/csv/spin_new tools/scripts/tune.sh replay spin_test >/dev/null
    tools/scripts/tune.sh compare base spin_new
EOF
}

case "${1:-}" in
  record)  shift; cmd_record "$@" ;;
  list|ls) cmd_list ;;
  replay)  shift; cmd_replay "$@" ;;
  analyze) shift; cmd_analyze "$@" ;;
  compare) shift; cmd_compare "$@" ;;
  clean)   shift; cmd_clean "$@" ;;
  ""|-h|--help|help) usage ;;
  *) die "未知子命令: $1（试试 tune.sh --help）" ;;
esac
