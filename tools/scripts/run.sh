#!/bin/bash
# ⭐⭐⭐ HZMIR 启动脚本（自动瞄准 / 打符）
#
# ## 为什么需要它（把踩过的坑固化进去）
# 1. ⚠️ **哨兵需要 ROS2**（导航桥），而 `~/.ros/log` 是**只读**的 →
#    直接跑会 `Failed opening file ... Read-only file system`。
#    本脚本自动把 `ROS_LOG_DIR`/`ROS_HOME` 指到**可写目录**。
# 2. ⚠️ **`sp_msgs` 的 typesupport 库**在 `ros2_ws/install` →
#    不 source 就报 `Type support not from this implementation`。
# 3. ⚠️ **热键只在 tty 下有效** —— 非 tty（管道/后台）会自动禁用，脚本会提示。
# 4. ⚠️ **构建目录自动选择**：优先 Release（性能），没有就退 Debug 并**警告**
#    （Debug 慢 3~5 倍，性能数据无意义）。
#
# 用法：
#   tools/scripts/run.sh infantry                 # 真机：步兵自瞄
#   tools/scripts/run.sh sentry                   # 真机：哨兵（自动 source ROS2）
#   tools/scripts/run.sh infantry --video=a.avi   # 零硬件：录像回放
#   tools/scripts/run.sh infantry --csv=run --record   # 顺便采数据
#   tools/scripts/run.sh hero --buff              # 打符
#   tools/scripts/run.sh infantry --watchdog      # 崩了自动重启
#
# 也支持：HZMIR_BIN_DIR 指定构建目录；HZMIR_DEBUG=1 强制用 Debug
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
WORKSPACE="${HZMIR_WORKSPACE:-$(cd "$ROOT/../.." && pwd)}"

if [ -t 1 ]; then B=$'\e[1m'; G=$'\e[32m'; Y=$'\e[33m'; R=$'\e[31m'; N=$'\e[0m'
else B=""; G=""; Y=""; R=""; N=""; fi
say(){ echo "${B}══ $* ══${N}"; }
ok(){ echo "  ${G}✅${N} $*"; }
warn(){ echo "  ${Y}⚠️${N} $*"; }
die(){ echo "  ${R}❌${N} $*" >&2; exit 1; }

usage() { sed -n '2,25p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; }

# ── 解析第一个位置参数 = 兵种 ──
ROBOT=""
case "${1:-}" in
  infantry|hero|sentry|uav) ROBOT="$1"; shift ;;
  -h|--help|"") usage; exit 0 ;;
  *) die "未知兵种 '$1'（可选: infantry hero sentry uav）" ;;
esac

# ── 其它开关（脚本自己的，不传给程序）──
USE_WATCHDOG=0; FORCE_DEBUG=0; PARAMS_OVERRIDE=""
EXTRA=()
for a in "$@"; do
  case "$a" in
    --watchdog) USE_WATCHDOG=1 ;;
    --debug-bin) FORCE_DEBUG=1 ;;
    --params=*) PARAMS_OVERRIDE="${a#--params=}" ;;
    *) EXTRA+=("$a") ;;
  esac
done

# ── ① 选构建目录 ──
say "选择构建目录"
BIN_DIR=""
if [ -n "${HZMIR_BIN_DIR:-}" ]; then
  BIN_DIR="$HZMIR_BIN_DIR"
elif [ "$FORCE_DEBUG" = 1 ] || [ "${HZMIR_DEBUG:-0}" = 1 ]; then
  BIN_DIR="$WORKSPACE/exp/hzmir_dbg"
elif [ -x "$WORKSPACE/exp/hzmir_build/src/$ROBOT" ]; then
  BIN_DIR="$WORKSPACE/exp/hzmir_build"
elif [ -x "$WORKSPACE/exp/hzmir_dbg/src/$ROBOT" ]; then
  BIN_DIR="$WORKSPACE/exp/hzmir_dbg"
  warn "**没有 Release 构建** → 退回 Debug"
  warn "Debug 慢 3~5 倍，⚠️ **性能数字无意义**（本项目三次栽在 Debug/Release 混用）"
else
  die "找不到可执行 → 先跑 tools/scripts/build.sh"
fi

BIN="$BIN_DIR/src/$ROBOT"
[ -x "$BIN" ] || die "找不到 $BIN"

# 校验构建类型（目录名 vs CMakeCache）
BT="$(grep -m1 '^CMAKE_BUILD_TYPE:' "$BIN_DIR/CMakeCache.txt" 2>/dev/null | cut -d= -f2 || echo '?')"
DIRN="$(basename "$BIN_DIR" | tr 'A-Z' 'a-z')"
if case "$DIRN" in *dbg*|*debug*) true;; *) false;; esac; then
  [ "$BT" = "Debug" ] || die "$BIN_DIR 名字暗示 Debug 但类型是 $BT（assert 会失效！）"
fi
ok "$BIN  [$BT]"

# ── ② 参数文件 ──
PARAMS="${PARAMS_OVERRIDE:-$ROOT/params/$ROBOT.yaml}"
if [ -f "$PARAMS" ]; then ok "参数: $PARAMS"
else warn "找不到 $PARAMS（程序会用默认值）"; fi

# ── ③ ROS2（哨兵必须）；含 ROS_LOG_DIR 可写目录修复 ──
if [ "$ROBOT" = "sentry" ]; then
  say "ROS2 环境（哨兵需要导航桥）"
  [ -f /opt/ros/humble/setup.bash ] || die "找不到 /opt/ros/humble/setup.bash"
  # shellcheck disable=SC1091
  set +u; source /opt/ros/humble/setup.bash; set -u
  ROS_PREFIX="${HZMIR_ROS_PREFIX:-$WORKSPACE/ros2_ws/install}"
  if [ -f "$ROS_PREFIX/setup.bash" ]; then
    # shellcheck disable=SC1091
    set +u; source "$ROS_PREFIX/setup.bash"; set -u
    ok "已 source /opt/ros/humble + $ROS_PREFIX（sp_msgs 可用）"
  else
    warn "找不到 $ROS_PREFIX/setup.bash → 可能报 'Type support not from this implementation'"
  fi
  # ⚠️ 关键修复：~/.ros/log 常常只读 → 指到可写目录
  export ROS_LOG_DIR="$ROOT/output/logs/ros"   # ⭐ W87
  export ROS_HOME="$ROOT/output/logs/roshome"
  mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
  ok "ROS_LOG_DIR=$ROS_LOG_DIR  （⚠️ 修 ~/.ros/log 只读的坑）"
fi

# ── ③.5 串口预检（只说结论；细节见 README「零硬件运行」）──
if [[ ! " ${EXTRA[*]-} " =~ " --video" ]]; then
  COM_PORT="$(grep -m1 '^com_port:' "$PARAMS" 2>/dev/null | sed 's/.*"\(.*\)".*/\1/' || true)"
  COM_PORT="${COM_PORT:-/dev/gimbal}"
  if [ -e "$COM_PORT" ]; then
    ok "串口 $COM_PORT 存在"
  else
    warn "找不到串口 $COM_PORT → 自动降级为虚拟下位机（无 IMU，EKF/命中不可信）"
  fi
fi


# ── ③.6 `/dev/video*` 预检已删：本项目用工业相机 SDK，不是 V4L2（见 README）──

# ── ③.7 ⭐⭐⭐ 相机占用预检（海康相机【不独占】，MVS 能同时打开但抢流！）──
if [[ ! " ${EXTRA[*]-} " =~ " --video" ]] && [[ ! " ${EXTRA[*]-} " =~ " --video=" ]]; then
  # ⚠️ 用 `cut` 而不是 `sed`（sed 的贪婪匹配在这里会返回空，实测踩到）
  CNAME="$(grep -m1 '^camera_name:' "$PARAMS" 2>/dev/null | cut -d: -f2 | tr -d ' "' || true)"
  if [ "${CNAME:-}" = "hikrobot" ] || [ "${CNAME:-}" = "mindvision" ]; then
    say "相机占用预检（${CNAME}）"
    # ⚠️ 用 **可执行文件名** 匹配，不用 `pgrep -f`（后者会误报命令行里恰好含 "MVS" 的进程，
    #    比如本脚本自己、或 `grep MVS`）—— 实测踩到过误报。
    RIVALS=""
    for _pid in $(pgrep -f 'MVS|MvCamera' 2>/dev/null); do
      [ "$_pid" = "$$" ] && continue
      _exe="$(readlink -f "/proc/$_pid/exe" 2>/dev/null || true)"
      _base="$(basename "${_exe:-}" 2>/dev/null || true)"
      case "$_base" in
        *MVS*|*MvCamera*|*mvviewer*|*MvViewer*)
          RIVALS="${RIVALS}${_pid} ${_exe}
" ;;
      esac
    done
    RIVALS="${RIVALS%$'\n'}"
    if [ -n "$RIVALS" ]; then
      warn "⚠️ **发现可能占用工业相机的进程**："
      echo "$RIVALS" | sed 's/^/        /'
      echo
      warn "海康 USB 相机**不独占** —— MVS 客户端能和本程序**同时 OpenDevice**，"
      warn "但**只有一个能真正取到流**。若程序报 \`0x80000007\`（无数据）→ **就是它！**"
      echo "        建议先执行：  pkill -f MVS"
      echo
      if [ "${HZMIR_ALLOW_CAM_CONFLICT:-0}" != 1 ]; then
        echo "  （5 秒后继续；Ctrl-C 停下去关掉它们，或 HZMIR_ALLOW_CAM_CONFLICT=1 跳过此等待）"
        sleep 5
      fi
    else
      ok "没发现占用相机的进程（MVS 等）"
    fi
  fi
fi

# ── ④ tty / 热键 提示 ──
say "运行"
if [ -t 0 ]; then
  ok "stdin 是 tty → 热键可用"
  echo "      [2]CSV  [3]PlotJuggler  [1]窗口(L3)  [4]存图(L3)"
  echo "      [d]日志级别  [p]暂停  [r]热重载配置  [q]退出"
else
  warn "stdin 非 tty → **热键已自动禁用**（要用热键请直接在终端跑）"
fi
echo "      Ctrl-C 退出"
echo

# ── ⑤ 组装命令 ──
CMD=("$BIN")
[ -f "$PARAMS" ] && CMD+=("$PARAMS")
CMD+=("${EXTRA[@]+"${EXTRA[@]}"}")

if [ "$USE_WATCHDOG" = 1 ]; then
  W="$ROOT/tools/scripts/watchdog.sh"
  [ -x "$W" ] || die "找不到 watchdog.sh"
  ok "watchdog 模式（崩溃自动重启）"
  exec "$W" "${CMD[@]}"
else
  exec "${CMD[@]}"
fi
