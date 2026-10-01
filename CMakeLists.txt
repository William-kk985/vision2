cmake_minimum_required(VERSION 3.16)
project(hzmir_vision CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ⭐⭐⭐ W36：**按目录名核对构建类型**
#   本项目已三次栽在「`assert` 在 Release 是空操作」，为此确立「双构建跑同一套 ctest」。
#   但 W35 清 CMakeCache 重配时**漏传 `-DCMAKE_BUILD_TYPE=Debug`**，
#   顶层回退成 Release → 目录叫 `hzmir_dbg`（名字暗示 Debug）却是 **Release**，
#   而 ctest 依旧 "17/17 全绿" —— **纪律被绕过却毫无提示**。
#   ⇒ 若构建目录名含 `dbg`/`debug`，就**强制要求** Debug 类型。
get_filename_component(_hzm_bindir_name "${CMAKE_BINARY_DIR}" NAME)
string(TOLOWER "${_hzm_bindir_name}" _hzm_bindir_lc)
if(_hzm_bindir_lc MATCHES "dbg|debug")
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(FATAL_ERROR
      "构建目录 '${_hzm_bindir_name}' 的名字暗示 Debug，但 CMAKE_BUILD_TYPE='${CMAKE_BUILD_TYPE}'。\n"
      "  ⇒ 这会让 `assert` 变成空操作，**整个 ctest 结果不可信**。\n"
      "  ⇒ 修法：rm -f ${CMAKE_BINARY_DIR}/CMakeCache.txt && "
      "cmake -S ${CMAKE_SOURCE_DIR} -B ${CMAKE_BINARY_DIR} -DCMAKE_BUILD_TYPE=Debug ...\n"
      "  （本项目已三次栽在这个坑上，故设为硬失败）")
  endif()
endif()

# ⭐⭐ W35：ROS2 导航桥（可选）
#   ⚠️ 修复前 `io/CMakeLists.txt` 写的是 `if(HAS_ROS2)` —— 那是 **config.hpp 里的 C++ 宏**，
#      CMake 里**从来没有这个变量** → **io/ros2/ 从未被编译过**（静默失效）。
#   现在：CMake 用 `HZMIR_WITH_ROS2`；两侧加一致性检查（见 config.hpp 的互斥段）。
# ⭐⭐ **config.hpp 是唯一宏入口** —— 于是 CMake 不该让人手动同步，而是**直接读它的意图**。
#   下面从 config.hpp 里抓 `#define HAS_ROS2`；抓到就自动打开本 option。
#   （仍保留 `-DHZMIR_WITH_ROS2=ON/OFF` 手动覆盖的能力）
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/config.hpp" _hzm_ros2_line
     REGEX "^[ \t]*#[ \t]*define[ \t]+HAS_ROS2")
if(_hzm_ros2_line)
  set(_hzm_cfg_wants_ros2 ON)
  message(STATUS "  config.hpp: HAS_ROS2 已定义 → 自动启用 ROS2 构建")
else()
  set(_hzm_cfg_wants_ros2 OFF)
endif()
option(HZMIR_WITH_ROS2 "启用 ROS2 导航桥（需要 ament/rclcpp/std_msgs/sp_msgs）" ${_hzm_cfg_wants_ros2})
if(HZMIR_WITH_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sp_msgs REQUIRED)
  add_compile_definitions(HZMIR_CMAKE_HAS_ROS2)   # → config.hpp 的一致性检查会用到

  # ⭐ W39：把「ament 前缀」和「ROS2 库路径」传给测试环境，
  #   让 `ctest` **不依赖外部 source** 也能跑 ROS2 测试
  #   （rclcpp 运行期要靠 AMENT_PREFIX_PATH 找 sp_msgs 的 RMW 类型支持）
  get_filename_component(_hzm_sp_msgs_prefix "${sp_msgs_DIR}/../../.." ABSOLUTE)
  set(HZMIR_AMENT_PREFIX_PATH "${_hzm_sp_msgs_prefix}:$ENV{AMENT_PREFIX_PATH}")
  # 收集 sp_msgs 与 ROS2 的 lib 目录
  set(HZMIR_ROS2_LD_PATH "$ENV{LD_LIBRARY_PATH}")
  foreach(_p "${_hzm_sp_msgs_prefix}/lib" "${sp_msgs_DIR}/../../../lib")
    if(EXISTS "${_p}")
      set(HZMIR_ROS2_LD_PATH "${_p}:${HZMIR_ROS2_LD_PATH}")
    endif()
  endforeach()
  message(STATUS "  sp_msgs:   ${_hzm_sp_msgs_prefix}")
  message(STATUS "  ROS2:      ✅ 开启（ament/rclcpp/sp_msgs）")
else()
  message(STATUS "  ROS2:      ⬜ 未开启（-DHZMIR_WITH_ROS2=ON 可开）")
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(nlohmann_json REQUIRED)

# ⭐ include root = 项目根 → 让 #include "core/types.hpp" / "config.hpp" 可用
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

enable_testing()   # ⭐ 让 ctest 能发现 test/ 下的用例

# ⭐⭐ W19：**强制包含 `config.hpp`** —— 宏规范①「唯一宏入口」的编译期保证。
#    修复前：`config.hpp` 谁都没 include → 那些互斥 `#error` 检查从未参与编译。
add_compile_options(-include ${CMAKE_CURRENT_SOURCE_DIR}/config.hpp)

add_subdirectory(third_party)
add_subdirectory(core)
add_subdirectory(drivers)
add_subdirectory(io)
add_subdirectory(utils)
add_subdirectory(src)
add_subdirectory(test)

message(STATUS "──────────────── hzmir_vision 配置 ────────────────")
message(STATUS "  平台:      ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "  OpenCV:    ${OpenCV_VERSION}")
message(STATUS "  功能组:    ENABLE_AUTO_AIM / ENABLE_AUTO_BUFF")
message(STATUS "  相机后端:  HAS_WEBCAM")
message(STATUS "───────────────────────────────────────────────────")
