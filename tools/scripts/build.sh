#!/bin/bash
# ⭐⭐⭐ HZMIR 编译脚本
#
# ## 为什么需要它（把踩过的坑固化进去）
# 1. ⚠️ **构建类型纪律**：本项目已**三次**栽在「`assert` 在 Release 是空操作」。
#    所以「目录名含 dbg ⇒ 必须 Debug」这条要在**脚本层**再保证一次
#    （CMakeLists 已有一道硬失败，这里是第二道）。
# 2. ⚠️ **`config.hpp` 改了要重跑 cmake**（`HAS_ROS2` 等宏是 CMake 读取的，
#    只 `make` 不会重新配置 → 宏静默不生效）。
# 3. ⚠️ **ROS2 的 `LD_LIBRARY_PATH`**：`sp_msgs` 的类型支持库在 `ros2_ws/install`，
#    不 source 就会 `Type support not from this implementation`。
#
# 用法：
#   tools/scripts/build.sh                  # Release（默认）
#   tools/scripts/build.sh --debug          # Debug
#   tools/scripts/build.sh --both           # 两个都编（跑双构建 ctest 前用）
#   tools/scripts/build.sh --clean          # 删掉构建目录重来
#   tools/scripts/build.sh -j8              # 指定并行数
#   tools/scripts/build.sh --test           # 编完跑 ctest
set -euo pipefail

# ── 路径（脚本在 tools/scripts/ 下 → 项目根在 ../..）──
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

# ── 默认值 ──
MODE="release"
CLEAN=0
JOBS="$(nproc)"
RUN_TEST=0
WORKSPACE="${HZMIR_WORKSPACE:-$(cd "$ROOT/../.." && pwd)}"   # exp/ 所在的工作区

# ── 参数 ──
while [ $# -gt 0 ]; do
  case "$1" in
    --debug)  MODE="debug" ;;
    --release) MODE="release" ;;
    --both)   MODE="both" ;;
    --clean)  CLEAN=1 ;;
    --test)   RUN_TEST=1 ;;
    -j*)      JOBS="${1#-j}" ;;
    -h|--help)
      sed -n '2,25p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
      exit 0 ;;
    *) echo "未知参数: $1（用 --help）" >&2; exit 2 ;;
  esac
  shift
done

# ── 颜色 ──
if [ -t 1 ]; then B=$'\e[1m'; G=$'\e[32m'; Y=$'\e[33m'; R=$'\e[31m'; N=$'\e[0m'
else B=""; G=""; Y=""; R=""; N=""; fi
say()  { echo "${B}══ $* ══${N}"; }
ok()   { echo "  ${G}✅${N} $*"; }
warn() { echo "  ${Y}⚠️${N} $*"; }
die()  { echo "  ${R}❌${N} $*" >&2; exit 1; }

# ── ① 环境自检 ──
say "环境自检"
for c in cmake g++; do command -v "$c" >/dev/null || die "找不到 $c"; done
ok "cmake $(cmake --version | head -1 | awk '{print $3}')  ·  $(g++ --version | head -1 | awk '{print $1,$NF}')"

# ROS2（可选）：config.hpp 里 HAS_ROS2 开时才需要
HAS_ROS2_CFG=0
grep -qE '^[[:space:]]*#[[:space:]]*define[[:space:]]+HAS_ROS2' config.hpp && HAS_ROS2_CFG=1
ROS_PREFIX="${HZMIR_ROS_PREFIX:-$WORKSPACE/ros2_ws/install}"
if [ "$HAS_ROS2_CFG" = 1 ]; then
  ok "config.hpp: HAS_ROS2 已开 → 需要 ROS2"
  [ -f /opt/ros/humble/setup.bash ] || die "找不到 /opt/ros/humble/setup.bash"
  # shellcheck disable=SC1091
  set +u; source /opt/ros/humble/setup.bash; set -u
  if [ -f "$ROS_PREFIX/setup.bash" ]; then
    # shellcheck disable=SC1091
    set +u; source "$ROS_PREFIX/setup.bash"; set -u
    ok "已 source: /opt/ros/humble + $ROS_PREFIX"
  else
    warn "找不到 $ROS_PREFIX/setup.bash（sp_msgs 可能缺失 → 哨兵会报 typesupport 错）"
  fi
else
  ok "config.hpp: HAS_ROS2 未开 → 跳过 ROS2（哨兵也能编，但没有导航桥）"
fi

# ── ② config.hpp 变更检测 ──
say "config.hpp 变更检测"
CFG_HASH_FILE="$ROOT/.cache/config_hash"
mkdir -p "$ROOT/.cache"
NEW_HASH="$(md5sum config.hpp | cut -d' ' -f1)"
if [ -f "$CFG_HASH_FILE" ] && [ "$(cat "$CFG_HASH_FILE")" != "$NEW_HASH" ]; then
  warn "config.hpp 变了 → **强制重新 configure**（否则宏静默不生效）"
  CLEAN=2   # 标记：只重跑 cmake，不删整个目录
fi

# ── ③ 构建 ──
build_one() {
  local kind="$1"                    # release | debug
  local dir="$WORKSPACE/exp/hzmir_build"
  local type="Release"
  [ "$kind" = "debug" ] && { dir="$WORKSPACE/exp/hzmir_dbg"; type="Debug"; }

  say "构建 $type → $dir"

  # ⚠️ 第二道纪律：目录名和类型必须一致（CMakeLists 里是第一道硬失败）
  case "$(basename "$dir" | tr 'A-Z' 'a-z')" in
    *dbg*|*debug*)
      [ "$type" = "Debug" ] || die "目录名暗示 Debug，但类型是 $type —— 会导致 assert 失效" ;;
  esac

  if [ "$CLEAN" = 1 ] && [ -d "$dir" ]; then rm -rf "$dir"; ok "已删除旧目录（--clean）"; fi

  local cfg_args=(-S "$ROOT" -B "$dir" "-DCMAKE_BUILD_TYPE=$type")
  [ -n "${CMAKE_PREFIX_PATH:-}" ] && cfg_args+=("-DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH")

  # config.hpp 变了 or 目录不存在 → 都要 configure
  if [ ! -f "$dir/CMakeCache.txt" ] || [ "$CLEAN" = 2 ]; then
    [ "$CLEAN" = 2 ] && rm -f "$dir/CMakeCache.txt"
    cmake "${cfg_args[@]}" > "$dir.cmake.log" 2>&1 || {
      echo; tail -30 "$dir.cmake.log" >&2; die "cmake 配置失败（日志: $dir.cmake.log）"; }
    ok "cmake 配置完成"
  else
    ok "已有 CMakeCache → 跳过 configure（改 config.hpp 会自动触发）"
  fi

  if ! cmake --build "$dir" -j"$JOBS" > "$dir.build.log" 2>&1; then
    echo; grep -E "error:" "$dir.build.log" | head -20 >&2
    die "编译失败（完整日志: $dir.build.log）"
  fi

  local n; n="$(find "$dir" -maxdepth 3 -type f -executable -newer "$dir/CMakeCache.txt" 2>/dev/null | wc -l)"
  ok "$type 编译完成（$n 个可执行）"

  if [ "$RUN_TEST" = 1 ]; then
    say "跑 ctest（$type）"
    # ⚠️ 清掉 shell 里 source 进来的 ROS 环境，让 ctest 按 CMake 里配置的 ENVIRONMENT 走
    ( cd "$dir" && env -u AMENT_PREFIX_PATH -u LD_LIBRARY_PATH ctest --output-on-failure 2>&1 \
        | tail -5 | sed 's/^/  /' ) || die "ctest 有失败"
  fi
}

case "$MODE" in
  release) build_one release ;;
  debug)   build_one debug ;;
  both)    build_one release; build_one debug ;;
esac

echo "$NEW_HASH" > "$CFG_HASH_FILE"

# ── ④ 汇总 ──
say "产物"
for d in "$WORKSPACE/exp/hzmir_build" "$WORKSPACE/exp/hzmir_dbg"; do
  [ -d "$d" ] || continue
  t="$(grep -m1 '^CMAKE_BUILD_TYPE:' "$d/CMakeCache.txt" 2>/dev/null | cut -d= -f2)"
  printf "  %-42s %s\n" "$d" "${t:-?}"
  for m in infantry hero sentry uav; do
    [ -x "$d/src/$m" ] && printf "      · %s\n" "$(basename "$d")/src/$m"
  done
done
echo
ok "下一步：tools/scripts/run.sh --help"
