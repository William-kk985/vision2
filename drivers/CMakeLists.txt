# ═══════════════════════════════════════════════════════════════════════════
# drivers/ —— 厂商 SDK 与硬件死绑（含 SDK 头 + .so）
#
# ⭐⭐⭐ W101（原 F7）：**每个 SDK 独立探测，缺了就不编**。
#
# ## 改前的问题
# 原来**无条件**编译全部四个驱动 + **无条件链接** `MvCameraControl MVSDK`
# ⇒ ⚠️ **没装海康/迈德威视 SDK 的机器，`cmake` 能过、`make` 直接链接失败**，
#   而且报错信息晦涩（`cannot find -lMvCameraControl`）。
#   而 `usbcamera` / `dm_imu` **不依赖任何厂商 SDK**（OpenCV / 纯逻辑），
#   本可以单独编 —— 却被一起拖死。
#
# ## 现在
# 逐个探测「头文件 + .so 在不在」：
#   · 在  → 编该驱动 + 链接其 SDK + 定义 `HZMIR_HAS_XXX`
#   · 不在 → ⭐ **跳过**（打一条 STATUS 说明），其余驱动照常
# ⚠️ 代价：`io/camera/camera.cpp`（**装配点**）里要按 `HZMIR_HAS_XXX` 决定
#    注册哪个相机 —— 那是宏规范**允许** `#ifdef` 的 4 处之一。
# ⭐ 好处：只有海康相机的队伍**不必装迈德威视 SDK**（反之亦然）；
#    手头没相机的开发机也能编过（只剩 `usbcamera`/`video` 路径）。
# ═══════════════════════════════════════════════════════════════════════════

# ── 按平台选 .so 目录 ──
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(HZMIR_SDK_ARCH amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(HZMIR_SDK_ARCH arm64)
else()
  message(FATAL_ERROR "不支持的架构: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(_hzm_drv_srcs)
set(_hzm_drv_libs)
set(_hzm_drv_defs)   # ⭐ 收集编译期宏，最后用 target_compile_definitions(PUBLIC) 广播

# ── ① 海康 MVS（HikRobot）──
set(_hik_inc "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/include/MvCameraControl.h")
set(_hik_lib "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/${HZMIR_SDK_ARCH}/libMvCameraControl.so")
if(EXISTS "${_hik_inc}" AND EXISTS "${_hik_lib}")
  list(APPEND _hzm_drv_srcs hikrobot/hikrobot.cpp)
  list(APPEND _hzm_drv_libs MvCameraControl)
  list(APPEND _hzm_drv_defs HZMIR_HAS_HIKROBOT)
  set(HZMIR_HAS_HIKROBOT ON)   # ⭐ 同时给 CMake 自己用（下面的"至少一个后端"检查）
  message(STATUS "  drivers: ✅ hikrobot（海康 MVS SDK 已就位）")
else()
  message(STATUS "  drivers: ⬜ hikrobot 跳过（缺 ${_hik_lib}，装海康 MVS 后重新 cmake）")
endif()

# ── ② 迈德威视（MindVision）──
set(_mv_inc "${CMAKE_CURRENT_SOURCE_DIR}/mindvision/include/CameraApi.h")
set(_mv_lib "${CMAKE_CURRENT_SOURCE_DIR}/mindvision/lib/${HZMIR_SDK_ARCH}/libMVSDK.so")
if(EXISTS "${_mv_inc}" AND EXISTS "${_mv_lib}")
  list(APPEND _hzm_drv_srcs mindvision/mindvision.cpp)
  list(APPEND _hzm_drv_libs MVSDK)
  list(APPEND _hzm_drv_defs HZMIR_HAS_MINDVISION)
  set(HZMIR_HAS_MINDVISION ON)
  message(STATUS "  drivers: ✅ mindvision（迈德威视 SDK 已就位）")
else()
  message(STATUS "  drivers: ⬜ mindvision 跳过（缺 ${_mv_lib}）")
endif()

# ── ③ USB 摄像头（OpenCV V4L2，无厂商 SDK）──
list(APPEND _hzm_drv_srcs usbcamera/usbcamera.cpp)
list(APPEND _hzm_drv_defs HZMIR_HAS_USBCAMERA)
set(HZMIR_HAS_USBCAMERA ON)

# ── ④ DM IMU（纯串口协议，无厂商 SDK）──
list(APPEND _hzm_drv_srcs dm_imu/dm_imu.cpp)
list(APPEND _hzm_drv_defs HZMIR_HAS_DM_IMU)
set(HZMIR_HAS_DM_IMU ON)

# ⚠️ 至少要有一个相机后端，否则程序没有任何取图途径
if(NOT HZMIR_HAS_HIKROBOT AND NOT HZMIR_HAS_MINDVISION AND NOT HZMIR_HAS_USBCAMERA)
  message(FATAL_ERROR "没有任何相机后端可用（海康/迈德威视 SDK 都缺，USB 也被排除？）")
endif()

# ── 组装 ──
add_library(hzmir_drivers STATIC ${_hzm_drv_srcs})

# ⭐ include 目录按需加（缺 SDK 时加进去也无害，但保持干净）
set(_hzm_drv_incs)
if(HZMIR_HAS_HIKROBOT)
  list(APPEND _hzm_drv_incs "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/include")
  list(APPEND _hzm_drv_incs "${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/${HZMIR_SDK_ARCH}")
endif()
if(HZMIR_HAS_MINDVISION)
  list(APPEND _hzm_drv_incs "${CMAKE_CURRENT_SOURCE_DIR}/mindvision/include")
  list(APPEND _hzm_drv_incs "${CMAKE_CURRENT_SOURCE_DIR}/mindvision/lib/${HZMIR_SDK_ARCH}")
endif()
target_include_directories(hzmir_drivers PUBLIC ${_hzm_drv_incs})
target_link_directories(hzmir_drivers PUBLIC ${_hzm_drv_incs})

# ⭐⭐⭐ 关键：用 **PUBLIC** 广播宏 —— `io/` 通过链接 `hzmir_drivers` 就能拿到
#   `HZMIR_HAS_HIKROBOT` 等（`add_compile_definitions` 是**目录作用域**，
#   在 drivers/ 里定义的话 io/ **看不到** —— 实测踩到过：`CameraParam` 未声明）。
target_compile_definitions(hzmir_drivers PUBLIC ${_hzm_drv_defs})

target_link_libraries(hzmir_drivers PUBLIC
  ${_hzm_drv_libs} usb-1.0 serial yaml-cpp ${OpenCV_LIBS} hzmir_utils)
