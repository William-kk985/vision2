#!/bin/bash
# ⭐⭐⭐ HZMIR 相机复位 —— 清除「红灯常亮」（相机被留在 grabbing 状态）
#
# ## 红灯是什么
# 海康 USB3 相机的灯语：
#   | 灯 | 含义 |
#   |---|---|
#   | **蓝灯** | 已上电、**未被取流**（idle，正常待机） |
#   | ⚠️ **红灯** | **正在取流（grabbing）/ 被占用** |
#
# ## 为什么会变红
# 程序调 `MV_CC_StartGrabbing` → **红灯**；
# 正常退出应调 `MV_CC_StopGrabbing` + `CloseDevice` + `DestroyHandle` → **回蓝**。
# ⚠️ 若 StopGrabbing 失败（或 handle 未初始化）→ **灯不灭**。
#   本项目 W56 已修：`capture_stop()` 改成「尽力清理不中途 return」+ 析构显式清理。
#   但**已经卡住**的相机需要本脚本复位（USB reset 会强制重新枚举设备）。
#
# 用法：
#   tools/scripts/camera-reset.sh          # 自动找海康（2bdf）
#   tools/scripts/camera-reset.sh 2bdf 0001
set -u

VID="${1:-2bdf}"
PID="${2:-0001}"

if [ -t 1 ]; then B=$'\e[1m'; G=$'\e[32m'; Y=$'\e[33m'; R=$'\e[31m'; N=$'\e[0m'
else B=""; G=""; Y=""; R=""; N=""; fi
say(){ echo "${B}══ $* ══${N}"; }
ok(){ echo "  ${G}✅${N} $*"; }
warn(){ echo "  ${Y}⚠️${N} $*"; }
bad(){ echo "  ${R}❌${N} $*"; }

echo "${B}相机复位${N}  目标 VendorID=0x$VID ProductID=0x$PID"
echo

say "1. 前置检查"
if ! lsusb 2>/dev/null | grep -qi "$VID:$PID"; then
  bad "lsusb 里找不到 $VID:$PID —— 相机没插 / 没上电"
  echo "      先插好相机再跑本脚本"
  exit 1
fi
ok "lsusb 找到设备"
if [ ! -d /dev/bus/usb ]; then
  bad "/dev/bus/usb 不存在 → 无法 USB reset"
  echo "      修法: sudo mkdir -p /dev/bus/usb && sudo mount -t usbfs none /dev/bus/usb"
  exit 1
fi
ok "/dev/bus/usb 存在"

say "2. ⚠️ 先确认没有程序占着相机"
OCC=$( (pgrep -af "MVS|MvCamera|hzmir|infantry|hero|sentry|uav" 2>/dev/null || true) | grep -v grep || true)
if [ -n "$OCC" ]; then
  warn "这些进程可能在用相机："
  echo "$OCC" | sed 's/^/      /'
  echo "      ⚠️ 不先关掉，reset 后还会被再次 grabbing → 灯又变红"
else
  ok "没发现占用进程"
fi

say "3. 执行 USB reset（会强制重新枚举设备）"
python3 - "$VID" "$PID" <<'PY' 2>&1 | sed 's/^/  /'
import ctypes, ctypes.util, sys, os, fcntl, time
vid = int(sys.argv[1], 16); pid = int(sys.argv[2], 16)
lib = ctypes.util.find_library("usb-1.0") or "/opt/MVS/lib/64/libusb-1.0.so.0"
u = ctypes.CDLL(lib)
u.libusb_init.argtypes = [ctypes.c_void_p]
u.libusb_open_device_with_vid_pid.restype = ctypes.c_void_p
u.libusb_open_device_with_vid_pid.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]
u.libusb_reset_device.argtypes = [ctypes.c_void_p]
u.libusb_close.argtypes = [ctypes.c_void_p]
u.libusb_exit.argtypes = [ctypes.c_void_p]
ctx = ctypes.c_void_p()
if u.libusb_init(ctypes.byref(ctx)) != 0:
    print("❌ libusb_init 失败"); raise SystemExit(1)
h = u.libusb_open_device_with_vid_pid(ctx, vid, pid)
if not h:
    print(f"❌ libusb 打不开设备 0x{vid:04x}:0x{pid:04x}")
    print("   → 可能被别的进程占着（先关掉 MVS / 我们的程序）")
    print("   → 或权限不足（检查 udev 规则 + 运行: sudo udevadm control --reload-rules && sudo udevadm trigger）")
    u.libusb_exit(ctx); raise SystemExit(1)
print("✅ 打开设备成功（说明权限 OK）")
r = u.libusb_reset_device(h)
if r == 0:
    print("✅ **USB reset 成功** —— 设备已重新枚举，红灯应该灭了（变蓝）")
else:
    print(f"⚠️ libusb_reset_device 返回 {r}")
    print("   → 再试：拔掉相机 5 秒再插上（最可靠）")
u.libusb_close(h); u.libusb_exit(ctx)
PY

say "4. 验证"
sleep 2
if lsusb 2>/dev/null | grep -qi "$VID:$PID"; then
  ok "reset 后仍能看到设备（正常）"
else
  warn "reset 后设备暂时消失 —— 等几秒会自动回来（重新枚举）"
fi
echo
echo "${B}══ 如果灯还是红的 ══${N}"
echo "  ⚠️ 说明还有进程占着它。按顺序试："
echo "    1) 关掉 MVS 客户端 / 所有跑我们程序的终端"
echo "    2) 再跑一次本脚本"
echo "    3) 还不行 → **物理拔插**（最可靠）"
echo
echo "  ⭐ 之后跑程序，**正常退出（Ctrl-C / 按 q）**，W56 的修复会正确 StopGrabbing → 灯回蓝"
