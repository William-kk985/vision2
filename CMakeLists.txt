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

# ⭐⭐ ROS2 导航桥（可选）—— ⭐⭐ W96：**CMake 自动检测，不需要 config.hpp**
#
#   改前的两层麻烦：
#     ① `config.hpp` 里 `#define HAS_ROS2` 是人手开关
#     ② CMake 用 `file(STRINGS)` 反向读那个 C++ 宏，再拿它当 CMake 变量的默认值
#     ③ 两侧还要一致性 `#error` 检查
#   ⚠️ 后果：**只要有人打开 config.hpp 的 HAS_ROS2，而机器上没有 `sp_msgs`，
#       cmake 就【直接失败】**（`find_package(sp_msgs REQUIRED)`）——
#       一个"编译期配置"却让"配置阶段"崩掉。
#
#   现在：**先静默探测 `sp_msgs`，装了才默认开**。
#     · 装了 sp_msgs（同济工作空间）→ 自动 ON，无需任何手动步骤
#     · 没装                        → 自动 OFF，构建照常
#     · 想强制：`-DHZMIR_WITH_ROS2=ON/OFF`
find_package(sp_msgs QUIET)
if(sp_msgs_FOUND)
  set(_hzm_ros2_default ON)
  message(STATUS "  检测到 sp_msgs → 默认启用 ROS2 导航桥")
else()
  set(_hzm_ros2_default OFF)
endif()
option(HZMIR_WITH_ROS2 "启用 ROS2 导航桥（需要 ament/rclcpp/std_msgs/sp_msgs）" ${_hzm_ros2_default})

# ═══════════════════════════════════════════════════════════════════════════
# ⭐⭐⭐ W104：**算法实验开关的自动解析**（`core/debug.hpp` §二）
#
# ## 为什么要自动解析（而不是每次手改 CMakeLists）
# 一个实验要动三处：① `core/debug.hpp` 加开关 ② `test/function/exp_*.cpp` 写实现
# ③ CMake 把那个 .cpp 挂进构建 + 定义宏。
# ⚠️ 第 ③ 步最容易**忘**（忘了就链接不到符号，报错还不好懂）。
# ⇒ 让 CMake **直接读 `core/debug.hpp`**：定义了哪个 `HZMIR_EXP_*`，
#   就自动 ① 全项目定义该宏 ② 把 `test/function/exp_<小写名字>.cpp` 加进构建。
#
# ⭐ 与已删除的 `config.hpp` 的区别（那次删对了，这次不一样）：
#   旧 `config.hpp` 的宏**啥也没干**（实测只有 1 个真在用）⇒ 解析它是**纯装饰**。
#   这里的 `HZMIR_EXP_*` **真的决定编不编** ⇒ 自动解析是**消除一个易错手步骤**。
# ═══════════════════════════════════════════════════════════════════════════
set(HZMIR_EXP_DEFS)
set(HZMIR_EXP_SRCS)
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/core/debug.hpp" _hzm_exp_lines
     REGEX "^[ \t]*#[ \t]*define[ \t]+HZMIR_EXP_[A-Za-z0-9_]+")
foreach(_line IN LISTS _hzm_exp_lines)
  string(REGEX MATCH "HZMIR_EXP_[A-Za-z0-9_]+" _name "${_line}")
  list(APPEND HZMIR_EXP_DEFS "${_name}")
  # ⭐ 约定：宏 `HZMIR_EXP_FOO_BAR` → 实现文件 `test/function/exp_foo_bar.cpp`
  string(TOLOWER "${_name}" _low)
  string(REPLACE "hzmir_exp_" "exp_" _file "${_low}")
  set(_src "${CMAKE_CURRENT_SOURCE_DIR}/test/function/${_file}.cpp")
  if(EXISTS "${_src}")
    list(APPEND HZMIR_EXP_SRCS "${_src}")
    message(STATUS "  🧪 实验开关: ${_name}  →  test/function/${_file}.cpp")
  else()
    message(STATUS "  🧪 实验开关: ${_name}  ⚠️ 找不到 test/function/${_file}.cpp（只定义宏）")
  endif()
endforeach()
if(HZMIR_EXP_DEFS)
  add_compile_definitions(${HZMIR_EXP_DEFS})
  message(STATUS "  🧪 实验模式：【已开启】—— ⚠️ 这不是同济默认行为，验证完请转正或删除")
else()
  message(STATUS "  🧪 实验模式：⬜ 全关（= 同济默认）")
endif()

if(HZMIR_WITH_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sp_msgs REQUIRED)
  add_compile_definitions(HZMIR_WITH_ROS2)   # ⭐ W96：代码里 #if defined(HZMIR_WITH_ROS2)

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

# ⭐ include root = 项目根 → 让 #include "core/types.hpp" 可用
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

enable_testing()   # ⭐ 让 ctest 能发现 test/ 下的用例

# ⭐ W96：`config.hpp` 已删除 —— 不再需要"强制包含宏入口"那套机制

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
