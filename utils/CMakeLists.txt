# utils/ —— 基础层：通用工具 + 轮子（★ 零业务依赖）
add_library(hzmir_utils STATIC
  math/math_tools.cpp
  math/crc.cpp
  log/logger.cpp
  log/log_filter.cpp
  concurrency/exiter.cpp
  ekf/extended_kalman_filter.cpp
  debug/img_tools.cpp
  debug/recorder.cpp
  debug/csv_sink.cpp
  debug/plotjuggler_sink.cpp
  debug/hotkeys.cpp
  debug/window_sink.cpp
  debug/image_sink.cpp
  control/pid.cpp
  wheels/fit/ransac_sine_fitter.cpp
  wheels/ballistic/rk4_ballistic.cpp
  wheels/ballistic/ballistic_table.cpp
  wheels/ballistic/linear_drag_ballistic.cpp
  wheels/detect/tgd.cpp
  wheels/estimate/rls_bullet_speed.cpp
  ov/device.cpp
    system/host_info.cpp
    system/thread_tuning.cpp
    system/disk_guard.cpp
    system/paths.cpp
)
target_link_libraries(hzmir_utils PUBLIC
  ${OpenCV_LIBS} fmt::fmt spdlog::spdlog yaml-cpp
  nlohmann_json::nlohmann_json pthread)

# ⭐ OpenVINO（可选）：device 解析轮子需要
find_package(OpenVINO QUIET COMPONENTS Runtime)
if(OpenVINO_FOUND)
  target_link_libraries(hzmir_utils PUBLIC openvino::runtime)
endif()
