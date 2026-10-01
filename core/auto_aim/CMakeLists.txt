# core/auto_aim/ —— 自瞄功能组（9 个角色 + 编排）
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(hzmir_auto_aim STATIC
  # ── 识别器 ──
  detector/detector.cpp
  detector/yolo.cpp
  detector/yolos/yolov5.cpp
  detector/yolos/yolov8.cpp
  detector/yolos/yolo11.cpp
  # ── 分类器 ──
  classifier/classifier.cpp
  # ── 坐标变换器 ──
  solver/solver.cpp
  # ── 跟踪器 ──
  tracker/tracker.cpp
  tracker/voter.cpp
  tracker/filter.cpp
  tracker/priority.cpp
  tracker/nav_bridge.cpp
  target/target_plugins_factory.cpp
  # ── 估计器 ──
  target/target.cpp
  # ── 规划器（MPC + legacy 槽位）──
  planner/mpc.cpp
  planner/legacy.cpp
  # ── 弹道 ──
  trajectory/ideal.cpp
  trajectory/rk4_drag.cpp
  trajectory/trajectory.cpp
  trajectory/table.cpp
  trajectory/tongji_linear.cpp
  # ── 开火器 ──
  shooter/shooter.cpp
  # ── 控制器 ──
  controller/gimbal_ctrl.cpp
)

target_link_libraries(hzmir_auto_aim PUBLIC
  hzmir_core hzmir_io hzmir_utils
  openvino::runtime tinympcstatic
  ${OpenCV_LIBS})
