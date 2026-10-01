#!/bin/bash
# ⭐ W15：进程崩溃自动重启（搬自同济 origin/auto_aim_hero:watchdog.sh，改进版）
#
# 同济原版问题：
#   · `cd /home/rm/Desktop/sp_vision_25` 硬编码
#   · 用 `pidof` 判断，多实例时会误判
#   · 崩溃无退避，可能疯狂重启刷日志
# 本版：路径自适应 + 指数退避 + 重启原因记录
#
# 用法： tools/scripts/watchdog.sh <可执行文件> [参数...]
#   e.g. tools/scripts/watchdog.sh ./hero configs/hero.yaml
set -u

BIN="${1:?用法: watchdog.sh <可执行文件> [参数...]}"
shift || true

MAX_RETRY="${MAX_RETRY:-100}"
RETRY=0
BACKOFF=1

cleanup() { echo "[watchdog] 收到终止信号，杀掉子进程"; pkill -P $$; exit 1; }
trap cleanup SIGINT SIGTERM

echo "[watchdog] 监视: $BIN $*  （上限 $MAX_RETRY 次重启）"

while [ "$RETRY" -lt "$MAX_RETRY" ]; do
  START=$(date +%s)
  "$BIN" "$@"
  CODE=$?
  END=$(date +%s)

  if [ "$CODE" -eq 0 ]; then
    echo "[watchdog] 正常退出（exit 0），不再重启"
    exit 0
  fi

  RETRY=$((RETRY + 1))
  RAN=$((END - START))
  echo "[watchdog] ⚠️ 第 $RETRY/$MAX_RETRY 次崩溃：exit=$CODE，本次存活 ${RAN}s"

  # 存活够久 → 当作偶发崩溃，退避重置
  if [ "$RAN" -ge 30 ]; then BACKOFF=1; else BACKOFF=$((BACKOFF * 2)); [ "$BACKOFF" -gt 16 ] && BACKOFF=16; fi
  echo "[watchdog] ${BACKOFF}s 后重启..."
  sleep "$BACKOFF"
done

echo "[watchdog] ❌ 已达重启上限 $MAX_RETRY，放弃"
exit 1
