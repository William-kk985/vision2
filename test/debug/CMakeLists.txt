# test/debug/ —— 一次性算法调试程序（doc 09 §14.11：不进主链路）
#   planner_offline：离线跑 MPC（★ 不需要相机 / OpenVINO 推理 / 录像）
add_executable(planner_offline planner_offline.cpp)
target_link_libraries(planner_offline hzmir_auto_aim hzmir_utils ${OpenCV_LIBS})
