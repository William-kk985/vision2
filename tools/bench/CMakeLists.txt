# ⭐ 效率对比 harness：本项目 vs 同济上游
#
# 用法（从【本文件所在目录】执行）：
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build -j
#   ./build/bench_tongji <同济 configs/demo.yaml> <录像>     # 需先编上游 auto_aim
#   ./build/bench_ours    <本项目 params/robots/X.yaml> <录像> [nostats]
#   ./build/probe_img     <本项目 config> <录像>             # 两条取图路径对比
#   ./build/probe_throttle <本项目 config> <录像>            # 验证 CPU 热降频
#
# ⚠️ 测量纪律（见 docs/20）：必须【交替执行】两边，否则 CPU 热降频会让结论反向。
cmake_minimum_required(VERSION 3.16)
project(hzmir_bench CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

# ── 本项目 ──
set(OURS  ${CMAKE_CURRENT_SOURCE_DIR}/../..)          # hzmir_vision/
set(HB    ${OURS}/../../exp/hzmir_build)              # 本项目的构建目录（可用 -DHB= 覆盖）
if(NOT EXISTS ${HB}/core/auto_aim/libhzmir_auto_aim.a)
  message(FATAL_ERROR "找不到本项目静态库：${HB}\n请先构建本项目，或用 -DHB=<build 目录> 指定。")
endif()
set(OURS_LIBS
  ${HB}/io/libhzmir_io.a
  ${HB}/drivers/libhzmir_drivers.a
  ${HB}/core/auto_aim/libhzmir_auto_aim.a
  ${HB}/core/libhzmir_core.a
  ${HB}/utils/libhzmir_utils.a
  ${HB}/third_party/tinympc/libtinympcstatic.a)

foreach(t bench_ours probe_img probe_throttle)
  add_executable(${t} ${t}.cpp)
  target_include_directories(${t} PRIVATE ${OURS} ${OURS}/third_party)
  target_link_libraries(${t} ${OURS_LIBS}
    ${OpenCV_LIBS} fmt::fmt spdlog::spdlog yaml-cpp openvino::runtime)
endforeach()

# ── 同济上游（可选：未 clone 上游时跳过）──
set(SP ${OURS}/../upstream/sp_vision_25)
include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${SP})
if(EXISTS ${SP}/tasks/auto_aim/yolo.cpp)
  add_library(tj_tools OBJECT
    ${SP}/tools/logger.cpp ${SP}/tools/img_tools.cpp ${SP}/tools/math_tools.cpp
    ${SP}/tools/extended_kalman_filter.cpp ${SP}/tools/trajectory.cpp)
  target_include_directories(tj_tools PRIVATE ${SP})
  add_library(tj_det OBJECT
    ${SP}/tasks/auto_aim/armor.cpp ${SP}/tasks/auto_aim/classifier.cpp
    ${SP}/tasks/auto_aim/detector.cpp ${SP}/tasks/auto_aim/yolo.cpp
    ${SP}/tasks/auto_aim/yolos/yolov5.cpp ${SP}/tasks/auto_aim/yolos/yolov8.cpp
    ${SP}/tasks/auto_aim/yolos/yolo11.cpp ${SP}/tasks/auto_aim/solver.cpp
    ${SP}/tasks/auto_aim/target.cpp ${SP}/tasks/auto_aim/tracker.cpp
    ${SP}/tasks/auto_aim/voter.cpp ${SP}/tasks/auto_aim/planner/planner.cpp)
  target_include_directories(tj_det PRIVATE ${SP})
  target_link_libraries(tj_det openvino::runtime)
  add_subdirectory(${SP}/tasks/auto_aim/planner/tinympc tinympc_build)
  target_link_libraries(tj_det tinympcstatic)
  add_executable(bench_tongji bench_tongji.cpp)
  target_include_directories(bench_tongji PRIVATE ${SP})
  target_link_libraries(bench_tongji tj_det tj_tools
    ${OpenCV_LIBS} fmt::fmt spdlog::spdlog yaml-cpp)
  message(STATUS "已找到同济上游，将编译 bench_tongji")
else()
  message(STATUS "未找到同济上游（${SP}）⇒ 跳过 bench_tongji")
endif()
