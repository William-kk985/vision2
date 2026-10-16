# core/auto_aim/ —— 自瞄功能组（⭐ W139：**6 个角色**，与设计文档 doc 09 §4 一致）
#
#   detector/    ① 识别器        cv::Mat     → list<Armor>
#   classifier/  ② 分类器        数字识别
#   solver/      ③ 坐标变换器    Armor(2D)   → Armor(3D)
#   tracker/     ④ 跟踪器        list<Armor> → list<Target>
#   target/      ⑤ 估计器        EKF 本体
#   planner/     ⑥ 规划器        Target      → Plan
#                （弹道 4 实现 + 开火判据都在 planner/ 里 —— 与哈工程 fire_control 同理）
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
  # ── 估计器 ──
  target/target.cpp
  # ── 规划器 ⑥（W139：弹道 4 实现 + 开火判据并入本角色）──
  planner/mpc.cpp            # MPC（IPlanner 实现之一）
  planner/legacy.cpp         # legacy Aimer（另一个实现）
  planner/trajectory.cpp     # 弹道槽位基类
  planner/ideal.cpp          # 弹道①：斜抛无阻力（同济解析解）
  planner/rk4_drag.cpp       # 弹道②：RK4 + 空气阻力
  planner/table.cpp          # 弹道③：查表
  planner/tongji_linear.cpp  # 弹道④：同济线性
  planner/shooter.cpp        # 开火判据（角度容差 + 距离）
)

target_link_libraries(hzmir_auto_aim PUBLIC
  hzmir_core hzmir_io hzmir_utils
  openvino::runtime tinympcstatic
  ${OpenCV_LIBS})
