# test/ros2/ —— ROS2 通信层测试（仅 HZMIR_WITH_ROS2=ON 时参与构建）
if(NOT HZMIR_WITH_ROS2)
  message(STATUS "test/ros2/: HZMIR_WITH_ROS2 未开启 → 跳过")
  return()
endif()

add_executable(test_ros2_roundtrip test_ros2_roundtrip.cpp)
target_link_libraries(test_ros2_roundtrip
  hzmir_io hzmir_auto_aim hzmir_utils
  rclcpp::rclcpp ${std_msgs_TARGETS} ${sp_msgs_TARGETS})
target_include_directories(test_ros2_roundtrip PRIVATE ${sp_msgs_INCLUDE_DIRS})
add_test(NAME test_ros2_roundtrip COMMAND test_ros2_roundtrip)
# ⭐ rclcpp 需要可写的 ROS_LOG_DIR（默认 ~/.ros/log 在沙箱里只读）
# ⭐⭐ 让 ctest **不依赖外部 `source`** 也能跑：
#   rclcpp 运行期要靠 `AMENT_PREFIX_PATH` 找 `sp_msgs` 的 **RMW 类型支持**，
#   靠 `LD_LIBRARY_PATH` 找 `.so`。把「ament 前缀」和「当前 LD_LIBRARY_PATH」都注入。
#   `ament_prefix_path` 由顶层 CMakeLists 从 `sp_msgs_DIR` 推出。
set_tests_properties(test_ros2_roundtrip PROPERTIES
  ENVIRONMENT
    "ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_log;ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;AMENT_PREFIX_PATH=${HZMIR_AMENT_PREFIX_PATH};LD_LIBRARY_PATH=${HZMIR_ROS2_LD_PATH}")

# ⭐ W39：手动诊断工具（同济 publish_test / subscribe_test / topic_loop_test 的合并升级版）
add_executable(ros2_diag ros2_diag.cpp)
target_link_libraries(ros2_diag
  hzmir_io hzmir_utils
  rclcpp::rclcpp ${std_msgs_TARGETS} ${sp_msgs_TARGETS} ${OpenCV_LIBS})
target_include_directories(ros2_diag PRIVATE ${sp_msgs_INCLUDE_DIRS})
# ⚠️ **不注册成 ctest** —— 它是手动工具（无限循环），由人运行

add_executable(test_ros2_overhead test_ros2_overhead.cpp)
target_link_libraries(test_ros2_overhead hzmir_io hzmir_utils rclcpp::rclcpp ${std_msgs_TARGETS} ${sp_msgs_TARGETS})
target_include_directories(test_ros2_overhead PRIVATE ${sp_msgs_INCLUDE_DIRS})
add_test(NAME test_ros2_overhead COMMAND test_ros2_overhead)
set_tests_properties(test_ros2_overhead PROPERTIES
  ENVIRONMENT "ROS_LOG_DIR=${CMAKE_BINARY_DIR}/ros_log;ROS_HOME=${CMAKE_BINARY_DIR}/ros_home;AMENT_PREFIX_PATH=${HZMIR_AMENT_PREFIX_PATH};LD_LIBRARY_PATH=${HZMIR_ROS2_LD_PATH}")
