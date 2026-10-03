/**
 * @file src/uav.cpp
 * @brief 无人机主程序（参考同济 `src/uav.cpp`）
 *
 * ## 与其它兵种的区别
 * | 项 | uav | infantry | sentry |
 * |---|---|---|---|
 * | 检测 | ⭐ **`Detector`（传统方法）**，YOLO 被注释掉 | `YOLO` | `YOLO` |
 * | 档位 | `auto_aim` **+ `outpost`** 都走自瞄 | `auto_aim` | `auto_aim` |
 * | 开火 | `Shooter::shoot(...)`（旧开火器） | `planner.fire` | `Aimer` 内含 |
 * | 枪口 | — | — | `ShootMode`（左/右/双） |
 *
 * ⭐ **W34 新增**：`--video` 零硬件验证 + `--tongji` 兼容开关 + Debug CSV
 * ⭐ **W34 顺带修**：`io::Command buff_command;` 原来是**未初始化**的（E2 家族，已在契约层修）
 */
#include "config.hpp"   // ⭐ 唯一宏入口（宏规范 ①）

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera/camera.hpp"
#include "core/auto_aim/detector/det_stats.hpp"
#include "utils/system/disk_guard.hpp"
#include "utils/system/paths.hpp"   // ⭐ W87   // ⭐ W83
#include "utils/system/host_info.hpp"
#include "utils/system/thread_tuning.hpp"   // ⭐ W82   // ⭐ W81：本机核数 + 建议   // ⭐ W63
#include "drivers/dm_imu/dm_imu.hpp"
#include "core/auto_aim/planner/legacy.hpp"
#include "core/auto_aim/detector/detector.hpp"
#include "core/auto_aim/shooter/shooter.hpp"
#include "core/auto_aim/solver/solver.hpp"
#include "core/auto_aim/target/target_debug_fill.hpp"   // ⭐ W70
#include "core/auto_aim/tracker/tracker.hpp"
#include "core/auto_aim/detector/yolo.hpp"
#include "core/auto_buff/planner/buff_aimer.hpp"
#include "core/auto_buff/detector/detector.hpp"
#include "core/auto_buff/solver/buff_solver.hpp"
#include "core/auto_buff/target/buff_target.hpp"
#include "core/auto_buff/type.hpp"
#include "utils/concurrency/exiter.hpp"
#include "utils/debug/img_tools.hpp"
#include "utils/log/logger.hpp"
#include "utils/math/math_tools.hpp"
#include "utils/debug/plotter.hpp"
#include "utils/debug/recorder.hpp"
#include "io/board/replay.hpp"
#include "io/camera/video.hpp"
#include "utils/debug/csv_sink.hpp"
#include "utils/debug/debug_sink.hpp"
#include "utils/debug/debug_setup.hpp"   // ⭐ W49：一行装配
#include "utils/ov/device.hpp"

#ifndef HZMIR_CONFIG_HPP
#  error "src/uav.cpp 必须先 #include \"config.hpp\"（宏规范 ①：唯一宏入口）"
#endif

const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{@config-path   | params/uav.yaml | yaml配置文件路径 }"
  "{video v        | | ⭐ 录像路径（给了就走录像回放，无需硬件）}"
  "{video-speed    | 1.0 | ⭐ 录像播放倍速}"
  "{force-mode     | 1 | ⭐ 录像模式档位：0=idle 1=auto_aim 2=small_buff 3=big_buff 4=outpost}"
  "{bullet-speed   | 22.0 | 录像模式下的弹速}"
  "{csv            | | ⭐ Debug CSV 输出前缀}"
  "{pj             | false | ⭐ 是否发 PlotJuggler UDP（跨机器请看 --pj-host）}"
  "{pj-host        | 127.0.0.1 | ⭐ PlotJuggler 目标 IP（跨机器时填对方 IP）}"
  "{pj-port        | 9870 | ⭐ PlotJuggler 目标端口}"
  "{record         | false | ⭐⭐ 录像到 output/video/（默认**不录**；录会占一个核做 MJPG 编码）}"
  "{tongji         | true | ⭐⭐ 同济兼容模式（默认 true = 完全同济行为）}"
  "{strict-device  | false | ⭐ 严格设备模式}"
  "{no-board       | false | ⭐⭐ 强制虚拟下位机（不碰串口；只有摄像头时用）}"
  "{strict-board   | false | ⭐⭐ 串口不存在就失败退出（同济行为）；默认自动降级虚拟板}"
  "{det-stats      | true | ⭐ 逐帧打印检测统计（候选→各步过滤）；false 硬关}"
  "{log-keep-days  | 30 | ⭐ 日志保留天数（超期自动清理 output/logs/；0=不清理）}";

using namespace std::chrono_literals;

namespace
{
// ⭐ W34：录像模式下的「下位机替身」，刻意模仿 `io::CBoard` 的公开形状
//   （`bullet_speed` / `mode` / `shoot_mode` / `imu_at` / `send(Command)`）
//   → 主循环写成模板即可**一份逻辑两种下位机**，不动 `io::CBoard`（W31）
struct ReplayCBoard
{
  double bullet_speed;
  io::Mode mode;
  io::ShootMode shoot_mode = io::ShootMode::both_shoot;
  io::ReplayBoard replay_;
  size_t sent_count = 0;

  ReplayCBoard(const std::string & pose_path, io::Mode m, double bs)
  : bullet_speed(bs), mode(m),
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

/// @brief 无人机主循环（⭐ 一份逻辑，`io::CBoard` / `ReplayCBoard` 各实例化一次）
template <typename Board>
int run_uav(io::CameraBase & camera, Board & cboard, const std::string & config_path,
            const std::string & csv_prefix, bool record,
            bool pj = false, const std::string & pj_host = "127.0.0.1", uint16_t pj_port = 9870)
{
  tools::Exiter exiter;
  // ⭐⭐ W48：录像**默认关**（录会占一个核做 MJPG 编码；要录传 --record）
  tools::Recorder recorder(30, record);

  auto_aim::Detector detector(config_path);   // ⭐ uav 用**传统检测器**（同济如此）
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  // ⭐ Debug 数据面
    // ⭐⭐ W49（方案 D）：**一行装配整个 Debug 体系**
    // ⭐⭐ W81：启动时打印「本机核数 + 建议配置」（核少时提示关 sink）
    // ⭐⭐ W83：录像磁盘守卫（**启动时一次**：配额清理 + 剩余空间 + 低空间警告）
    // ⚠️ 这两个兵种的 `cli` 不在本作用域 → 用固定配额（10 GB，与 infantry/hero 默认一致）
    // ⚠️ 这两个兵种的 `cli` 不在本作用域 → 用默认 30 天
    tools::guard_on_startup(30, tools::paths::video());   // ⭐ W84

    tools::DebugRuntime dbg({.csv_prefix = csv_prefix, .pj = pj, .pj_host = pj_host,
                             .pj_port = pj_port, .name = "uav"});
    auto & hub = dbg.hub;                 // ⭐ 别名：保持下游代码一字不改
    auto & expense = dbg.expense;
    auto & hotkeys = dbg.hotkeys;
    bool & paused = dbg.paused;

    cv::Mat img;
    std::chrono::steady_clock::time_point t;
  auto_aim::FrameDebug fd;
  uint32_t frame_id = 0;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    // W86: time the camera wait separately (blocking, not CPU work).
    expense.begin("cam_wait");
    camera.read(img, t);
    expense.end("cam_wait");

    expense.begin("perceive");
    if (img.empty()) break;   // ⭐ 录像读完 / 相机掉线 → 干净退出（W16 的教训）
    const Eigen::Quaterniond q = cboard.imu_at(t - 1ms);
    recorder.record(img, q, t);
    expense.end("perceive");

    const auto mode = cboard.mode;
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[static_cast<size_t>(mode)]);
      last_mode = mode;
    }

    fd.frame_id = frame_id++;
    fd.mode = static_cast<uint8_t>(mode);
    fd.t_cam_wait_us = expense.us("cam_wait");   // W86: blocking wait, not CPU
    fd.t_perceive_us = expense.us("perceive");
    io::Command command{};   // ⭐ W34：契约层已给默认值（原来是未初始化的）

    /// 自瞄（⚠️ uav 的 outpost 也走自瞄 —— 与同济一致）
    if (mode == io::Mode::auto_aim || mode == io::Mode::outpost) {
      solver.set_R_gimbal2world(q);
      const Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

      expense.begin("detect");
      auto armors = detector.detect(img);
      expense.end("detect");
      expense.begin("track");
      auto targets = tracker.track(armors, t);
      expense.end("track");

      command = aimer.aim(targets, t, cboard.bullet_speed);
      command.shoot = shooter.shoot(command, aimer, targets, ypr);

      fd.detector.armor_count = static_cast<int>(armors.size());
      // ⭐⭐ W64：**填上从没被赋值过的两列**（原来 CSV 里 det_best_conf / det_nms 永远是 0）
      {
        const auto & st = auto_aim::last_detect_stats();
        fd.detector.nms_survivors = st.nms_survivors;      // NMS 存活数（过滤前）
        fd.detector.best_confidence = st.best_conf;        // 最高置信度
      }
      fd.detector.t_infer_us = expense.us("detect");
      fd.tracker.t_track_us = expense.us("track");
      // ⭐⭐ W71：`sol_*` 四列（原来永远是 0）
      fd.solver = tracker.solver().last_debug();
      // ⭐⭐ W70：填 `tgt_*`（原来 `fd.target.*` 从没被赋值 → CSV 里 13 列永远 0）
      if (!targets.empty())
        auto_aim::fill_target_debug(fd.target, targets.front(), fd.solver.t_solve_us);
    }

    /// 打符
    else if (mode == io::Mode::small_buff || mode == io::Mode::big_buff) {
      const auto tb0 = std::chrono::steady_clock::now();
      buff_solver.set_R_gimbal2world(q);
      expense.begin("detect");
      auto power_runes = buff_detector.detect(img);
      expense.end("detect");
      buff_solver.solve(power_runes);

      fd.buff.rune_type = (mode == io::Mode::small_buff) ? 0 : 1;
      fd.buff.fanblade_count = power_runes ? static_cast<int>(power_runes->fanblades.size()) : 0;

      if (mode == io::Mode::small_buff) {
        buff_small_target.get_target(power_runes, t);
        fd.buff.spd = buff_small_target.spd;
        fd.buff.solved = !buff_small_target.is_unsolve();
        auto target_copy = buff_small_target;
        command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      } else {
        buff_big_target.get_target(power_runes, t);
        fd.buff.spd = buff_big_target.spd;
        fd.buff.solved = !buff_big_target.is_unsolve();
        auto target_copy = buff_big_target;
        command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      }
      fd.buff.t_us = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::steady_clock::now() - tb0)
                       .count();
      fd.detector.t_infer_us = expense.us("detect");
    }

    else {
      command = io::Command{};   // ⭐ W34：idle 时发**明确的零值**（原来是未初始化）
    }

    cboard.send(command);
    fd.controller.cmd_yaw = command.yaw;
    fd.controller.cmd_pitch = command.pitch;
    fd.controller.control = command.control;
    fd.controller.shoot = command.shoot;
    fd.t_frame_us = expense.us("perceive") + expense.us("detect") + expense.us("track");
    hub.on_frame(fd);
    expense.next_frame();
  }

  if (!csv_prefix.empty())
    tools::logger()->info("[uav] Debug CSV 已写出: {}_frames.csv", csv_prefix);
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
  const auto video_path = cli.get<std::string>("video");
  const auto csv_prefix = cli.get<std::string>("csv");

  // ⭐ 同济兼容开关
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

  if (video_path.empty()) {
    // ── 真实硬件：io::CBoard（CAN）+ io::Camera ──
    io::Camera camera(config_path);
    io::CBoard cboard(config_path);
    tools::logger()->info("[uav] 真实硬件模式（CBoard/CAN）");
    return run_uav(camera, cboard, config_path, csv_prefix, cli.get<bool>("record"),
            cli.get<bool>("pj"), cli.get<std::string>("pj-host"),
            static_cast<uint16_t>(cli.get<int>("pj-port")));
  }

  // ── 录像回放（零硬件）──
  std::string pose = video_path;
  const auto dot = pose.rfind('.');
  pose = (dot == std::string::npos) ? pose + ".txt" : pose.substr(0, dot) + ".txt";

  const int fm = cli.get<int>("force-mode");
  const io::Mode m = (fm >= 0 && fm <= 4) ? static_cast<io::Mode>(fm) : io::Mode::auto_aim;
  io::VideoCamera camera(video_path, cli.get<double>("video-speed"));
  ReplayCBoard cboard(pose, m, cli.get<double>("bullet-speed"));
  tools::logger()->info(
    "[uav] 录像回放模式: {} mode={}({})", video_path, int(m),
    (m >= 0 && m < int(io::MODES.size())) ? io::MODES[m] : "?");
  const int rc = run_uav(camera, cboard, config_path, csv_prefix, cli.get<bool>("record"),
            cli.get<bool>("pj"), cli.get<std::string>("pj-host"),
            static_cast<uint16_t>(cli.get<int>("pj-port")));
  tools::logger()->info("[uav] 录像播放完毕（共 {} 帧），退出", cboard.sent_count);
  return rc;
}
