/**
 * @file src/sentry.cpp
 * @brief 哨兵主程序（⭐ **单相机**版，参考同济 `src/sentry.cpp`）
 *
 * ## 与同济哨兵的区别
 * 详见 `docs/autoaim_compare/14-同济哨兵与本项目单相机哨兵对比.md`。摘要：
 *
 * | 项 | 同济哨兵 | 本文件 |
 * |---|---|---|
 * | 相机 | **4 个**（前 + 后 + 左USB + 右USB） | **1 个** |
 * | 全向搜索 | `decider.decide()` 轮询 3 相机 + **硬编码安装角**(62°/−62°/170°) | ❌ **去掉** |
 * | 多相机队列 | `Perceptron` + `DetectionResult` 队列 | ❌ 去掉（类型已提升到 `core/types.hpp`） |
 * | 4 项横切能力 | `decider.*` | ⭐ 已内聚进 `Tracker`（**默认关 = 同济行为**） |
 * | 自瞄解算 | **`Aimer`（legacy）** ⚠️ 不是 MPC | ✅ **同**（按同济） |
 * | 下位机 | `io::CBoard`（CAN）+ `ShootMode`（左/右/双枪口） | ✅ **同** |
 *
 * ## 设计：**一个循环，两个实例**
 * ```
 *   camera : io::CameraBase*   ← io::Camera（真实） / io::VideoCamera（录像）   [W10 已统一]
 *   board  : 模板参数          ← io::CBoard（真实 CAN） / ReplayCBoard（录像替身）
 * ```
 * ⚠️ `io::CBoard` 的 API 与 `io::Gimbal` 本质不同（见 W31 记录），
 *    **不**为了统一去改 `CBoard` —— 而是把主循环写成模板，让两种下位机各实例化一次。
 *
 * ## 零硬件验证
 * ```bash
 * ./src/sentry --video=demo.avi --force-mode=1 params/robots/sentry.yaml
 * ```
 */
#include "config.hpp"   // ⭐ 唯一宏入口（宏规范 ①）

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "core/auto_aim/detector/yolo.hpp"
#include "core/auto_aim/detector/det_stats.hpp"
#include "utils/system/disk_guard.hpp"
#include "utils/system/paths.hpp"   // ⭐ W87   // ⭐ W83
#include "utils/system/host_info.hpp"
#include "utils/system/thread_tuning.hpp"   // ⭐ W82   // ⭐ W81：本机核数 + 建议   // ⭐ W63
#include "core/auto_aim/planner/legacy.hpp"   // ⭐ 同济哨兵用 Aimer（legacy）
#include "core/auto_aim/solver/solver.hpp"
#include "core/auto_aim/target/target_debug_fill.hpp"   // ⭐ W70
#include "core/auto_aim/tracker/tracker.hpp"
#include "drivers/dm_imu/dm_imu.hpp"
#include "io/board/can/cboard.hpp"
#include "io/board/replay.hpp"
#include "io/camera/camera.hpp"
#include "io/camera/video.hpp"
#include "utils/concurrency/exiter.hpp"
#include "utils/debug/csv_sink.hpp"
#include "utils/debug/debug_sink.hpp"
#include "utils/debug/debug_setup.hpp"   // ⭐ W49：一行装配
#include "utils/debug/hotkeys.hpp"
#include "utils/debug/plotjuggler_sink.hpp"
#include "utils/debug/recorder.hpp"
#include "utils/log/log_filter.hpp"
#include "utils/log/logger.hpp"
#include "utils/ov/device.hpp"
#include "core/auto_aim/tracker/nav_bridge.hpp"   // ⭐ W35：上行目标信息

// ⭐⭐ W35：ROS2 导航桥（宏规范：`#ifdef` 只允许出现在 config.hpp / **装配函数** / CMakeLists / 测试入口
//    —— 本文件就是**装配点**，所以这里是合规位置）
#if defined(HAS_ROS2)
#  include "io/ros2/ros2.hpp"
#endif

#ifndef HZMIR_CONFIG_HPP
#  error "src/sentry.cpp 必须先 #include \"config.hpp\"（宏规范 ①：唯一宏入口）"
#endif

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | params/robots/sentry.yaml | yaml 配置文件路径}"
  "{video v        | | ⭐ 录像路径（给了就走录像回放，无需相机/下位机）}"
  "{video-speed    | 1.0 | ⭐ 录像播放速率（1.0=实时；调大=快放；⭐ 0=不节流全速跑批）}"
  "{force-mode     | 1 | ⭐ 录像模式档位：0=idle 1=auto_aim 2=small_buff 3=big_buff 4=outpost}"
  "{bullet-speed   | 22.0 | 录像模式下的弹速（下位机不可用时）}"
  "{dump-camera-params | false | ⭐ 只打印相机常用参数的当前值（用于把 MVS 里的好值抄进 yaml 的 camera_params）}"
  "{camera-config  | | ⭐ 覆盖相机配置路径（默认用兵种 yaml 的 camera_config 键）}"
  "{shoot-mode     | 2 | ⭐ 哨兵枪口：0=left 1=right 2=both}"
  "{csv            | | ⭐ Debug CSV 输出前缀}"
  "{record         | false | ⭐⭐ 录像到 output/video/（默认**不录**；录会占一个核做 MJPG 编码）}"
  "{pj             | false | ⭐ 是否发 PlotJuggler UDP}"
  "{pj-host        | 127.0.0.1 | ⭐ PlotJuggler 目标 IP（跨机器时填对方 IP）}"
  "{pj-port        | 9870 | ⭐ PlotJuggler 目标端口}"
  "{tongji         | true | ⭐⭐ 同济兼容模式（默认 true = 完全同济行为）}"
  "{strict-device  | false | ⭐ 严格设备模式（true = 设备不可用就抛异常）}"
  "{no-board       | false | ⭐⭐ 强制虚拟下位机（不碰串口；只有摄像头时用）}"
  "{strict-board   | false | ⭐⭐ 串口不存在就失败退出（同济行为）；默认自动降级虚拟板}"
  "{det-stats      | true | ⭐ 逐帧打印检测统计（候选→各步过滤）；false 硬关}"
  "{log-keep-days  | 30 | ⭐ 日志保留天数（超期自动清理 output/logs/；0=不清理）}"
  "{log-off        | | ⭐⭐ 按模块静音日志，逗号分隔（如 yolov5,VirtualBoard）}"
  "{log-only       | | ⭐⭐ 只打印这些模块（如 Tracker,Planner,Shooter）}";

using namespace std::chrono_literals;

namespace
{
// ═══════════════════════════════════════════════════════════════
// ⭐ 录像模式下的「下位机替身」：**刻意模仿 `io::CBoard` 的公开形状**
//   （`bullet_speed` / `mode` / `shoot_mode` / `imu_at(t)` / `send(Command)`）
//   → 主循环写成模板即可**一份逻辑两种下位机**，不必动 `io::CBoard`（W31）
// ═══════════════════════════════════════════════════════════════
struct ReplayCBoard
{
  double bullet_speed;
  io::Mode mode;
  io::ShootMode shoot_mode;
  io::ReplayBoard replay_;
  size_t sent_count = 0;

  ReplayCBoard(const std::string & pose_path, io::Mode m, double bs, io::ShootMode sm)
  : bullet_speed(bs), mode(m), shoot_mode(sm),
    replay_(pose_path, io::GimbalMode::AUTO_AIM, static_cast<float>(bs))
  {
  }

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point t) { return replay_.q(t); }

  void send(io::Command c)
  {
    ++sent_count;
    if (sent_count % 200 == 1)
      tools::logger()->debug("[ReplayCBoard] send#{:<6} control={} shoot={} yaw={:.4f} pitch={:.4f}",
                             sent_count, c.control, c.shoot, c.yaw, c.pitch);
  }
};

/// @brief 兵种程序主循环（⭐ 一份逻辑，`io::CBoard` / `ReplayCBoard` 各实例化一次）
template <typename Board>
int run_sentry(
  io::CameraBase & camera, Board & cboard, const auto_aim::Color /*enemy_color*/,
  const std::string & config_path, const std::string & csv_prefix, bool verbose_hotkeys,
  bool record, bool pj = false,
  const std::string & pj_host = "127.0.0.1", uint16_t pj_port = 9870)
{
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);   // ⭐ 同济哨兵用 legacy Aimer（不是 MPC）

  // ⭐⭐ W35：ROS2 导航桥（同济 `sentry.cpp` 的 `io::ROS2 ros2;`）
  //   构造函数里 `rclcpp::init` + 两个 spin 线程（发布/订阅各一）
#if defined(HAS_ROS2)
  auto ros2 = std::make_unique<io::ROS2>();
  tools::logger()->info("[sentry] ROS2 导航桥已启动（上行 auto_aim_target_pos / 下行 enemy_status+autoaim_target）");
#endif

  tools::Exiter exiter;
  // ⭐⭐ W48：录像**默认关**（录会占一个核做 MJPG 编码；要录传 --record）
  tools::Recorder recorder(30, record);

    // ⭐⭐ W49（方案 D）：**一行装配整个 Debug 体系**
    // ⭐⭐ W81：启动时打印「本机核数 + 建议配置」（核少时提示关 sink）
    // ⭐⭐ W83：录像磁盘守卫（**启动时一次**：配额清理 + 剩余空间 + 低空间警告）
    // ⚠️ 这两个兵种的 `cli` 不在本作用域 → 用固定配额（10 GB，与 infantry/hero 默认一致）
    // ⚠️ 这两个兵种的 `cli` 不在本作用域 → 用默认 30 天
    tools::guard_on_startup(30, tools::paths::video());   // ⭐ W84

      tools::DebugRuntime dbg({.csv_prefix = csv_prefix, .pj = pj, .pj_host = pj_host,
                               .pj_port = pj_port, .verbose_hotkeys = verbose_hotkeys,
                               .name = "sentry"});
    auto & hub = dbg.hub;                 // ⭐ 别名：保持下游代码一字不改
    auto & expense = dbg.expense;
    auto & hotkeys = dbg.hotkeys;
    bool & paused = dbg.paused;

  if (verbose_hotkeys) tools::logger()->info("{}", hotkeys.help());

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto_aim::FrameDebug fd;
  uint32_t frame_id = 0;

  while (!exiter.exit()) {
    hotkeys.poll();
    if (paused) {
      std::this_thread::sleep_for(20ms);
      continue;
    }

    // ⭐ 计时口径与 infantry 一致：`perceive` = 取图 + 姿态 + 记录 + 坐标变换
    // W86: time the camera wait separately (blocking, not CPU work).
    expense.begin("cam_wait");
    camera.read(img, t);
    expense.end("cam_wait");

    expense.begin("perceive");
    if (img.empty()) break;   // ⭐ 录像读完 / 相机掉线 → 干净退出（W16 的教训）

    const Eigen::Quaterniond q = cboard.imu_at(t - 1ms);
    recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);
    // ⭐ 在此结束 `perceive` —— **必须是正交分段**：
    //    若把 detect/track 也包进来，`budget_table` 会把嵌套区间相加 → 占比虚高（曾实测 158%）
    //    与 infantry 的口径保持一致
    expense.end("perceive");
    fd.t_cam_wait_us = expense.us("cam_wait");   // W86: blocking wait, not CPU
    fd.t_perceive_us = expense.us("perceive");

    if (cboard.mode == io::Mode::auto_aim) {
      expense.begin("detect");
      auto armors = yolo.detect(img);
      expense.end("detect");

      // ⭐⭐ 4 项横切能力（W7 已实现，**默认关闭 = 同济行为**）
      //   同济在 `sentry.cpp` 的顺序：
      //     get_invincible_armor → armor_filter → [get_auto_aim_target] → set_priority
      //   本项目把它们**内聚进 Tracker**：`set_invincible` / `set_auto_aim_targets` + yaml
      //   ⭐ W35：数据源（ROS2）现已接通
#if defined(HAS_ROS2)
      // ① 无敌状态（下行）—— 同济 `decider.get_invincible_armor(ros2.subscribe_enemy_status())`
      {
        const auto invincible_ids = ros2->subscribe_enemy_status();
        if (!invincible_ids.empty()) {
          // ⭐ 用**显式映射表**（`InvincibleMask::from_ids`）而不是同济的 `ArmorName(id-1)`
          //   —— 后者无边界检查，枚举一变就静默越界（W7 修复）
          const auto mask = auto_aim::InvincibleMask::from_ids(invincible_ids);
          tracker.set_invincible(mask);
          fd.tracker.invincible_count = static_cast<int>(invincible_ids.size());
        }
      }
      // ② 集火指令（下行）—— 同济默认注释掉，本项目默认也关（yaml 的 armor_filter 控制）
      {
        const auto ids = ros2->subscribe_autoaim_target();
        if (!ids.empty()) {
          std::vector<auto_aim::ArmorName> names;
          for (const auto id : ids) {
            if (id <= 0 || static_cast<size_t>(id) > auto_aim::ARMOR_NAMES.size()) {
              tools::logger()->warn("[sentry] 收到非法集火 id: {}", int(id));
              continue;
            }
            names.push_back(static_cast<auto_aim::ArmorName>(id - 1));   // 上行从 1 开始
          }
          fd.tracker.focus_target_count = static_cast<int>(names.size());
          tracker.set_auto_aim_targets(std::move(names));
        }
      }
#endif

      expense.begin("track");
      auto targets = tracker.track(armors, t);
      expense.end("track");

      // ⚠️ 同济哨兵在此处会走 `decider.decide(...)` 做 **4 相机全向搜索**；
      //    本项目**单相机** → 直接自瞄（与步兵一致）
      const io::Command command = aimer.aim(targets, t, cboard.bullet_speed, cboard.shoot_mode);
      cboard.send(command);

      // ⭐ 上行给导航（同济 `ros2.publish(decider.get_target_info(armors, targets))`）
      //   payload = {x, y, 1, ArmorName+1}，⚠️ 第 4 位从 **1** 开始
#if defined(HAS_ROS2)
      ros2->publish(auto_aim::target_info_for_nav(armors, targets));
#endif

      fd.frame_id = frame_id++;
      fd.mode = 1;
      fd.detector.armor_count = static_cast<int>(armors.size());
      // ⭐⭐ W64：**填上从没被赋值过的两列**（原来 CSV 里 det_best_conf / det_nms 永远是 0）
      {
        const auto & st = auto_aim::last_detect_stats();
        fd.detector.nms_survivors = st.nms_survivors;      // NMS 存活数（过滤前）
        fd.detector.best_confidence = st.best_conf;        // 最高置信度
      }
      fd.detector.t_infer_us = expense.us("detect");
      fd.tracker.armor_count = static_cast<int>(armors.size());
      fd.tracker.priority_mode = static_cast<int>(tracker.priority_mode());
      fd.tracker.t_track_us = expense.us("track");
      // ⭐⭐ W71：`sol_*` 四列（原来永远是 0）
      fd.solver = tracker.solver().last_debug();
      // ⭐⭐ W70：填 `tgt_*`（原来 `fd.target.*` 从没被赋值 → CSV 里 13 列永远 0）
      if (!targets.empty())
        auto_aim::fill_target_debug(fd.target, targets.front(), fd.solver.t_solve_us);
      fd.controller.cmd_yaw = command.yaw;
      fd.controller.cmd_pitch = command.pitch;
      fd.controller.control = command.control;
      fd.controller.shoot = command.shoot;
      hub.on_frame(fd);
    } else {
      cboard.send({false, false, 0, 0});
    }

    // ⭐ 帧总耗时 = 各正交段之和（budget_table 的假设）
    fd.t_frame_us = expense.us("perceive") + expense.us("detect") + expense.us("track");
    expense.next_frame();
  }

  if (!csv_prefix.empty())
    tools::logger()->info(
      "[sentry] Debug CSV 已写出: {}_frames.csv / {}_series.csv", csv_prefix, csv_prefix);
  return 0;
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }
  const auto config_path = cli.get<std::string>("@config-path");

  // ⭐⭐⭐ W95：**按模块过滤日志必须尽早设置** —— 否则启动期日志
  //   （如 `[infantry] 同济兼容模式` / `[VideoCamera]` / `[ReplayBoard]`）
  //   会在过滤生效前就打出来（实测踩过）。
  {
    const auto log_off = cli.get<std::string>("log-off");
    const auto log_only = cli.get<std::string>("log-only");
    if (!log_off.empty())  tools::set_log_modules_off(tools::parse_module_list(log_off));
    if (!log_only.empty()) tools::set_log_modules_only(tools::parse_module_list(log_only));
  }
  const auto video_path = cli.get<std::string>("video");
  const auto csv_prefix = cli.get<std::string>("csv");

  // ⭐ 同济兼容开关（集中控制硬编码偏差：E5 小陀螺判据 + E3b 阈值）
  {
    const bool tj = cli.get<bool>("tongji");
    auto_aim::Aimer::set_tongji_compat(tj);
    if (tj)
      tools::ExtendedKalmanFilter::use_tongji_nis_threshold();
    else
      tools::ExtendedKalmanFilter::use_chi2_q95_nis_threshold();
    tools::set_strict_device(cli.get<bool>("strict-device"));
    auto_aim::set_det_stats_enabled(cli.get<bool>("det-stats"));   // ⭐ W63
  }

  const auto enemy_color = auto_aim::Color::blue;   // ⭐ 同济 sentry.yaml 是 blue

  if (video_path.empty()) {
    // ── 真实硬件：io::CBoard（CAN）+ io::Camera —— 与同济哨兵一致 ──
    io::Camera camera(config_path, cli.get<bool>("dump-camera-params"), cli.get<std::string>("camera-config"));
    io::CBoard cboard(config_path);
    tools::logger()->info("[sentry] 真实硬件模式（CBoard/CAN）");
    return run_sentry(camera, cboard, enemy_color, config_path, csv_prefix, true, cli.get<bool>("record"),
                     cli.get<bool>("pj"), cli.get<std::string>("pj-host"),
                     static_cast<uint16_t>(cli.get<int>("pj-port")));
  }

  // ── 录像回放（零硬件）──
  std::string pose = video_path;
  const auto dot = pose.rfind('.');
  pose = (dot == std::string::npos) ? pose + ".txt" : pose.substr(0, dot) + ".txt";

  const int fm = cli.get<int>("force-mode");
  const io::Mode m = (fm >= 0 && fm <= 4) ? static_cast<io::Mode>(fm) : io::Mode::auto_aim;
  const int sm = cli.get<int>("shoot-mode");
  const io::ShootMode shoot_mode =
    (sm >= 0 && sm <= 2) ? static_cast<io::ShootMode>(sm) : io::ShootMode::both_shoot;

  io::VideoCamera camera(video_path, cli.get<double>("video-speed"));
  ReplayCBoard cboard(pose, m, cli.get<double>("bullet-speed"), shoot_mode);
  tools::logger()->info(
    "[sentry] 录像回放模式: {} mode={}({}) shoot_mode={}", video_path, int(m),
    (m >= 0 && m < int(io::MODES.size())) ? io::MODES[m] : "?",
    (shoot_mode == io::ShootMode::left_shoot    ? "left"
     : shoot_mode == io::ShootMode::right_shoot ? "right"
                                                : "both"));
  const int rc = run_sentry(camera, cboard, enemy_color, config_path, csv_prefix, true, cli.get<bool>("record"),
                     cli.get<bool>("pj"), cli.get<std::string>("pj-host"),
                     static_cast<uint16_t>(cli.get<int>("pj-port")));
  tools::logger()->info("[sentry] 录像播放完毕（共 {} 帧），退出", cboard.sent_count);
  return rc;
}
