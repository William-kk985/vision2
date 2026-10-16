# core/auto_aim/ —— 自瞄功能组（⭐ W139：**6 个角色**，与设计文档 doc 09 §4 一致）
#
#   detector/    ① 识别器        cv::Mat     → list<Armor>
#   classifier/  ② 分类器        数字识别
#   solver/      ③ 坐标变换器    Armor(2D)   → Armor(3D)
#   tracker/     ④ 跟踪器        list<Armor> → list<Target>
#   target/      ⑤ 估计器        EKF 本体
#   trajectory/  ⑥ 弹道          (v0,d,h)    → Trajectory（4 个实现）
#   planner/     ⑦ 规划器        Target      → Plan（MPC + legacy 两个实现）
#   shooter/     ⑧ 开火器        几何/距离    → 是否开火
#
# ⭐ W140：**规划与开火分开** ——
#   · `Planner`(MPC) **自带**开火判据（`plan.fire = traj_err < fire_thresh`）
#   · `Aimer`(legacy) **不带** ⇒ 需外挂 `Shooter`（只有 uav 走这条线）
#   ⇒ 两者是**不同职能**，不应混在一个目录
#   ⚠️ 这也正是设计文档 doc 09 §4 的划分（`trajectory/` 与 `shooter/` 各为独立角色）
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
  # ── 规划器 ⑦（W140：只装"规划"）──
  planner/mpc.cpp            # MPC（IPlanner 实现之一）
  planner/legacy.cpp         # legacy Aimer（⭐ 也是规划 —— 另一个 IPlanner 实现）
  # ── 弹道 ⑥（独立的角色，被规划器用）──
  trajectory/trajectory.cpp     # 槽位基类 + 工厂
  trajectory/ideal.cpp          # 弹道①：斜抛无阻力（同济解析解）
  trajectory/rk4_drag.cpp       # 弹道②：RK4 + 空气阻力
  trajectory/table.cpp          # 弹道③：查表
  trajectory/tongji_linear.cpp  # 弹道④：同济线性
  # ── 开火器 ⑧（⭐ 与规划分开：MPC 自带判据，legacy 不带 ⇒ 需外挂它）──
  shooter/shooter.cpp
)

target_link_libraries(hzmir_auto_aim PUBLIC
  hzmir_core hzmir_io hzmir_utils
  openvino::runtime tinympcstatic
  ${OpenCV_LIBS})
