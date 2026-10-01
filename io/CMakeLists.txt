# io/ —— 通信接口与协议（"我们能对硬件做什么"）
add_library(hzmir_io STATIC
  camera/camera.cpp
  camera/video.cpp
  board/replay.cpp
  board/can/cboard.cpp
  board/gimbal/gimbal.cpp
)
target_link_libraries(hzmir_io PUBLIC
  hzmir_drivers hzmir_utils serial yaml-cpp ${OpenCV_LIBS})

# ── ROS2 导航桥（可选）──────────────────────────────────────
# ⚠️ W35：这里原来写的是 `if(HAS_ROS2)`，而 `HAS_ROS2` 只是 config.hpp 的 C++ 宏 ——
#    CMake 里不存在该变量 → **io/ros2/ 从未被编译过**。现已改用 CMake option。
if(HZMIR_WITH_ROS2)
  target_sources(hzmir_io PRIVATE
    ros2/ros2.cpp
    ros2/publish2nav.cpp
    ros2/subscribe2nav.cpp)
  # ⚠️ W35：**不能用 `ament_target_dependencies`** —— 它内部走 `target_link_libraries`
  #    的 **plain 签名**，而上面已经用了 **keyword 签名**（`PUBLIC`），
  #    CMake 不允许混用 → 报 "All uses of target_link_libraries with a target must be
  #    either all-keyword or all-plain"。
  #    改为直接用 ament 导出的目标（`${pkg_TARGETS}` 是 ament 提供的变量）。
  target_link_libraries(hzmir_io PUBLIC
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${sp_msgs_TARGETS})
  target_include_directories(hzmir_io PUBLIC ${sp_msgs_INCLUDE_DIRS})
endif()
