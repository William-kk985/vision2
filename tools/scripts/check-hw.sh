#!/bin/bash
# ⭐⭐⭐ HZMIR 硬件诊断 —— 相机 / 串口 到底能不能被访问
#
# ## 为什么需要它
# 「**Not found camera!**」有两种完全不同的原因，必须区分：
#   | 现象 | 原因 |
#   |---|---|
#   | `MV_CC_EnumDevices` 返回 **0 个** | ⚠️ **libusb 枚举不到** → `/dev/bus/usb` 权限 / usbfs 未挂载 |
#   | `MV_CC_OpenDevice` 失败（有非 0 错误码） | ⚠️ 权限够但**被别的进程占着** |
# 光看程序日志分不清，所以单独做一个诊断。
#
# 用法： **在你的终端里**（不是沙箱/容器）跑
#   tools/scripts/check-hw.sh
set -u

if [ -t 1 ]; then B=$'\e[1m'; G=$'\e[32m'; Y=$'\e[33m'; R=$'\e[31m'; N=$'\e[0m'
else B=""; G=""; Y=""; R=""; N=""; fi
say(){ echo "${B}══ $* ══${N}"; }
ok(){ echo "  ${G}✅${N} $*"; }
warn(){ echo "  ${Y}⚠️${N} $*"; }
bad(){ echo "  ${R}❌${N} $*"; }

echo "${B}HZMIR 硬件诊断${N}   用户=$(whoami)  组=$(id -nG | tr ' ' ',')  架构=$(uname -m)"
echo

# ══════════ 一、USB 相机（海康 / 迈德威视） ══════════

say "1. USB 设备（lsusb）"
if command -v lsusb >/dev/null; then
  lsusb | sed 's/^/  /'
  echo
  if lsusb | grep -qi "2bdf"; then ok "找到海康设备（VendorID 2bdf）"
  else warn "lsusb 里**没有**海康（2bdf）—— 相机没插好 / 没上电 / 线不对"; fi
else
  warn "没装 usbutils（sudo apt install usbutils）"
fi

say "2. ⭐ usbfs 挂载（libusb 枚举的前提）"
if [ -d /dev/bus/usb ]; then
  ok "/dev/bus/usb 存在"
  n=$(find /dev/bus/usb -type f 2>/dev/null | wc -l)
  echo "      设备节点数: $n"
  find /dev/bus/usb -type f -printf '      %p  权限=%M  属主=%u:%g\n' 2>/dev/null | head -6
else
  bad "/dev/bus/usb **不存在** → libusb 一定枚举不到设备！"
  echo "      修法（需要 root）："
  echo "        sudo mkdir -p /dev/bus/usb"
  echo "        sudo mount -t usbfs none /dev/bus/usb"
  echo "      永久生效：检查 /etc/fstab 或 systemd 的 usb 挂载单元"
fi

say "3. ⭐ udev 规则（海康 = 2bdf）"
FOUND_RULE=0
for f in /etc/udev/rules.d/*2bdf* /etc/udev/rules.d/*hik* /etc/udev/rules.d/*MVS* /lib/udev/rules.d/*2bdf*; do
  [ -f "$f" ] || continue
  FOUND_RULE=1
  echo "  $f:"
  sed 's/^/      /' "$f"
done
if [ "$FOUND_RULE" = 1 ]; then
  ok "找到 udev 规则"
  echo "      ⚠️ 规则是**后加的**就一定要重新加载 + 重新插拔："
  echo "          sudo udevadm control --reload-rules && sudo udevadm trigger"
  echo "          （然后拔掉相机再插上）"
else
  warn "没有海康 udev 规则 —— 相机只有 root 能用"
  echo "      修法：新建 /etc/udev/rules.d/80-hikrobot.rules 内容："
  echo '        ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="2bdf", MODE="0666", GROUP="plugdev"'
  echo "      然后 sudo udevadm control --reload-rules && sudo udevadm trigger"
fi

say "4. libusb 能不能打开海康设备（关键判定）"
if command -v python3 >/dev/null; then
  python3 - <<'PY' 2>&1 | sed 's/^/  /'
import ctypes, ctypes.util
lib = ctypes.util.find_library("usb-1.0") or "/opt/MVS/lib/64/libusb-1.0.so.0"
try:
    u = ctypes.CDLL(lib)
except OSError as e:
    print(f"⚠️ 加载 libusb 失败: {e}"); raise SystemExit
u.libusb_init.argtypes = [ctypes.c_void_p]
ctx = ctypes.c_void_p()
if u.libusb_init(ctypes.byref(ctx)) != 0:
    print("❌ libusb_init 失败"); raise SystemExit
u.libusb_open_device_with_vid_pid.restype = ctypes.c_void_p
h = u.libusb_open_device_with_vid_pid(ctx, 0x2bdf, 0x0001)
u.libusb_close.argtypes = [ctypes.c_void_p]
u.libusb_exit.argtypes = [ctypes.c_void_p]
if h:
    print("✅ **libusb 能打开海康相机**（0x2bdf:0x0001）→ 权限没问题")
    u.libusb_close(h)
else:
    print("❌ **libusb 打不开海康相机** → 就是它导致 MV_CC_EnumDevices 返回 0！")
    print("   修法：① 上面第 2 步（usbfs 挂载）② 第 3 步（udev 规则 + reload + 重插）")
    print("   临时验证：用 sudo 跑一次程序，如果 sudo 下能找到相机 → 确认是权限问题")
u.libusb_exit(ctx)
PY
else
  warn "没有 python3，跳过"
fi

say "5. 海康 SDK"
for p in /opt/MVS/lib/64/libMvCameraControl.so \
         "$(dirname "$0")/../../drivers/hikrobot/lib/amd64/libMvCameraControl.so"; do
  if [ -f "$p" ]; then
    v=$(strings "$p" 2>/dev/null | grep -oE "^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$" | head -1)
    ok "$p  （版本 ${v:-?}）"
  else
    warn "找不到 $p"
  fi
done

# ══════════ 二、串口 ══════════
say "6. 串口设备"
avail="$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true)"
if [ -n "$avail" ]; then
  ok "可用串口: $(echo $avail | tr '\n' ' ')"
  for d in $avail; do
    [ -r "$d" ] && [ -w "$d" ] && echo "      $d 可读写 ✅" || echo "      $d ⚠️ 权限不足（加 udev 规则或加入 dialout 组）"
  done
else
  warn "没有 /dev/ttyUSB* / ttyACM* —— 下位机没插"
  echo "      程序会自动降级为**虚拟下位机**（W54），可以正常启动"
fi
if [ -e /dev/gimbal ]; then ok "/dev/gimbal 存在（udev 别名）"
else warn "/dev/gimbal 不存在（params/*.yaml 里的 com_port 指向它）"; fi
echo "  当前用户组: $(id -nG | tr ' ' ',')   （串口通常需要 dialout / plugdev）"

say "7. 摄像头（V4L2，本项目**不用**，仅供参考）"
ls /dev/video* 2>/dev/null | sed 's/^/  /' || echo "  （无 /dev/video*）"
echo "  ⚠️ 本项目用**工业相机 SDK**（海康 MV_CC / 迈德威视），不走 V4L2"
echo

echo "${B}══ 结论怎么用 ══${N}"
echo "  · 第 4 步 ❌ → 就是它！按提示修 usbfs / udev，然后**重新插拔相机**"
echo "  · 第 4 步 ✅ 但程序仍报 Not found camera → 相机被别的进程占着（关掉 MVS 客户端 / 别的程序）"
echo "  · 第 6 步 没有串口 → 正常，程序会自动用虚拟下位机（--strict-board 可改回失败）"
