# core/auto_buff/ —— 打符功能组（doc 09 §7.4：整块抄，类不拆）
#
# 角色（5 个）：
#   type.hpp/.cpp  符/扇叶/跟踪状态枚举 + FanBlade + PowerRune（契约性质，暂留本功能组）
#   detector/      Buff_Detector + yolo11_buff（内含「分类」FanBlade_type 与「丢帧计数」）
#   solver/        auto_buff::Solver（坐标变换器）
#   target/        Target / SmallTarget / BigTarget + buff_predict（内含「跟踪状态机」）
#   planner/       Aimer（mpc_aim → auto_aim::Plan；内含「开火判据」）
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(hzmir_auto_buff STATIC
  type.cpp
  detector/buff_detector.cpp
  detector/yolo11_buff.cpp
  solver/buff_solver.cpp
  target/buff_target.cpp
  planner/buff_aimer.cpp
)
target_link_libraries(hzmir_auto_buff PUBLIC
  hzmir_auto_aim hzmir_io hzmir_utils
  openvino::runtime
  ${OpenCV_LIBS})
