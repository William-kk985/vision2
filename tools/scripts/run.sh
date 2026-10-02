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
  export ROS_LOG_DIR="$ROOT/logs/ros"
  export ROS_HOME="$ROOT/logs/roshome"
  mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
  ok "ROS_LOG_DIR=$ROS_LOG_DIR  （⚠️ 修 ~/.ros/log 只读的坑）"
fi

# ── ③.5 ⭐⭐ 串口预检（没硬件时**提前**告知，别等程序崩）──
if [[ ! " ${EXTRA[*]-} " =~ " --video" ]] && [[ ! " ${EXTRA[*]-} " =~ " --video=" ]]; then
  say "串口预检（机器人在不在？）"
  COM_PORT="$(grep -m1 '^com_port:' "$PARAMS" 2>/dev/null | sed 's/.*"\(.*\)".*/\1/' || true)"
  COM_PORT="${COM_PORT:-/dev/gimbal}"
  if [ -e "$COM_PORT" ]; then
    ok "串口 $COM_PORT 存在"
  else
    warn "**找不到串口 $COM_PORT**（参数文件里的 com_port）"
    avail="$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ' || true)"
    if [ -n "$avail" ]; then
      warn "本机可用串口: $avail"
      warn "→ 改 params/$ROBOT.yaml 的 com_port，或用 udev 把下位机映射成 $COM_PORT"
    else
      warn "本机**没有任何** /dev/ttyUSB* / ttyACM* 设备（没接硬件）"
    fi
    echo
    ok "→ ⭐ **程序会自动降级为虚拟下位机**（W54，不再崩）—— 可以直接启动试试"
    echo "        ⚠️ 但虚拟板的 IMU 恒为单位四元数 → **EKF 预测/弹道/命中不可信**"
    echo
    echo "      其它跑法："
    echo "        tools/scripts/run.sh $ROBOT --video=录像.avi --force-mode=1   # 录像回放"
    echo "        tools/scripts/run.sh $ROBOT --no-board                        # 显式虚拟板"
    echo "        tools/scripts/run.sh $ROBOT --strict-board                    # 没下位机就失败（同济行为）"
    echo "        # --force-mode: 0=IDLE 1=自瞄 2=小符 3=大符"
    echo "        # 想看数据: 再加 --csv=run1  然后用 scripts/analyze.py 分析"
    echo
  fi
fi

# ── ③.6 ⭐ 摄像头预检（插了但没识别出来是常见问题）──
if [[ ! " ${EXTRA[*]-} " =~ " --video" ]] && [[ ! " ${EXTRA[*]-} " =~ " --video=" ]]; then
  if ls /dev/video* >/dev/null 2>&1; then
    ok "发现视频设备: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
    echo "        ⚠️ 但本项目用**工业相机 SDK**（海康 MV_CC / 迈德威视），不是 V4L2 /dev/video*"
    echo "           若程序报 'Not found camera!' → 那是 SDK 没找到相机，不是 /dev/video* 的问题"
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
