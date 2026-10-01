# drivers/ —— 厂商 SDK 与硬件死绑（含 SDK 头 + .so）
add_library(hzmir_drivers STATIC
  hikrobot/hikrobot.cpp
  mindvision/mindvision.cpp
  usbcamera/usbcamera.cpp
  dm_imu/dm_imu.cpp
)

target_include_directories(hzmir_drivers PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/include
  ${CMAKE_CURRENT_SOURCE_DIR}/mindvision/include
)

# 按平台选 .so 目录（同济的写法保留）
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(HZMIR_SDK_ARCH amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(HZMIR_SDK_ARCH arm64)
else()
  message(FATAL_ERROR "不支持的架构: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

target_link_directories(hzmir_drivers PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/lib/${HZMIR_SDK_ARCH}
  ${CMAKE_CURRENT_SOURCE_DIR}/mindvision/lib/${HZMIR_SDK_ARCH}
)

target_link_libraries(hzmir_drivers PUBLIC
  MvCameraControl MVSDK usb-1.0 serial yaml-cpp ${OpenCV_LIBS} hzmir_utils)
