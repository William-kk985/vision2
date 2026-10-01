/**
 * @file src/hero.cpp
 * @brief 英雄主程序（⭐ W15：参考同济 origin/auto_aim_hero 的 src/hero.cpp）
 *
 * 与步兵的区别：
 *   ① 弹道 = `table_42`（42mm 二次阻力查表）—— ⚠️ **不用**同济英雄的线性阻力模型
 *      （W14 实测：同济原式偏差 36.7 mrad，比完全不做阻力的 17.8 还差）
 *   ② ⭐ 端口/优先级：`bind_to_p_cores()` + `elevate_priority()`（同济写了但**注释掉了**）
 *   ③ `--watchdog` 提示配合 scripts/watchdog.sh 崩溃重启
 */
#include <sys/resource.h>
#include <sched.h>

#include <algorithm>
#include "config.hpp"   // ⭐ W19 修复：原来没 include → 整个宏体系从未生效

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <memory>
#include <mutex>
#include <thread>

#include "io/camera/camera.hpp"
#include "io/camera/video.hpp"        // ⭐ 录像回放相机
#include "io/board/board.hpp"         // ⭐ IBoard 统一接口
#include "io/board/gimbal/gimbal.hpp"   // ⭐ W31：io::Gimbal 已直接实现 IBoard
#include "io/board/replay.hpp"        // ⭐ 录像回放的「下位机」
#include "drivers/dm_imu/dm_imu.hpp"
// ⭐ 显式补上：同济 standard_mpc.cpp **没有** include yolo.hpp，
//    auto_aim::YOLO 是靠 mt_detector.hpp **传递**拿到的（隐性依赖，同 C23 一类）
#include "core/auto_aim/detector/yolo.hpp"
#include "core/auto_aim/solver/solver.hpp"
#include "core/auto_aim/planner/legacy.hpp"   // ⭐ W25：Aimer（含同济兼容开关）
#include "core/auto_aim/tracker/tracker.hpp"
// ⚠️ 同济 standard_mpc.cpp 里还有 aimer.hpp / commandgener.hpp / mt_detector.hpp / shooter.hpp
//    等 include，但**都没被使用**（死 include）。移植时已删除。
//    它们对应：legacy（Aimer）/ archive（multithread）/ shooter（旧开火，已被 planner.fire 取代）
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
// ⭐ W16：Debug 数据面（W8 建好，这次接进主程序）
#include "core/debug.hpp"
#include "utils/debug/csv_sink.hpp"
#include "utils/debug/hotkeys.hpp"   // ⭐ W18：终端热键
#include "utils/config/hot_reloader.hpp"   // ⭐ W21：配置热重载
#include "utils/ov/device.hpp"   // ⭐ W28：device 回退
#include "utils/yaml/yaml.hpp"
#include "utils/debug/image_sink.hpp"   // ⭐ W19：L3 存图
#include "utils/debug/window_sink.hpp"  // ⭐ W19：L3 可视化窗口
#include "utils/debug/debug_sink.hpp"
#include "utils/debug/debug_setup.hpp"   // ⭐ W49：一行装配
#include "utils/debug/plotjuggler_sink.hpp"

// ═══════════════════════════════════════════════════════════════
// ⭐ W15：进程调优（搬自同济 origin/auto_aim_hero:src/hero.cpp）
//   同济写了这两个函数但**都注释掉了**；这里做成 CLI 开关，默认开启
// ═══════════════════════════════════════════════════════════════

/// 按最高主频挑出 P 核并绑定（不硬编码核号 —— 同济原版硬编码 0~7）
static int bind_to_p_cores()
{
  namespace fs = std::filesystem;
  std::vector<std::pair<long, int>> cores;
  for (int cpu = 0; cpu < 256; ++cpu) {
    std::ifstream f("/sys/devices/system/cpu/cpu" + std::to_string(cpu) +
                    "/cpufreq/cpuinfo_max_freq");
    long khz = 0;
    if (f >> khz) cores.emplace_back(khz, cpu);
  }
  if (cores.empty()) {
    tools::logger()->warn("[hero] 读不到 cpufreq，跳过绑核");
    return 0;
  }
  std::sort(cores.rbegin(), cores.rend());
  const long top = cores.front().first;
  cpu_set_t mask;
  CPU_ZERO(&mask);
  int n = 0;
  for (const auto & [khz, cpu] : cores) {
    if (khz < top * 0.95) break;      // 只绑到最高频那档
    CPU_SET(cpu, &mask);
    ++n;
  }
  if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) == -1) {
    tools::logger()->warn("[hero] sched_setaffinity 失败: {}", std::strerror(errno));
    return 0;
  }
  tools::logger()->info("[hero] 已绑定 {} 个高性能核（最高频 {} kHz）", n, top);
  return n;
}

/// 提高调度优先级到 -20（需要 CAP_SYS_NICE；失败只警告不致命）
static void elevate_priority()
{
  if (setpriority(PRIO_PROCESS, 0, -20) == -1)
    tools::logger()->warn(
      "[hero] setpriority(-20) 失败: {}（需要 root 或 CAP_SYS_NICE，比赛时用 sudo 起）",
      std::strerror(errno));
  else
    tools::logger()->info("[hero] 调度优先级已提到 -20");
}

// ⭐ W19 编译期自检：确认 `config.hpp` 真的被包含（否则宏静默失效，就像修复前那样）
#ifndef HZMIR_CONFIG_HPP
#  error "src/hero.cpp 必须先 #include \"config.hpp\"（宏规范 ①：唯一宏入口）"
#endif

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }"
  "{video v        | | ⭐ 录像路径（.avi）；给了就走录像回放，无需相机/下位机}"
  "{video-speed    | 1.0 | ⭐ 录像播放倍速（跑批时调大）}"
  "{bullet-speed   | 22.0 | 录像模式下的弹速（下位机不可用时）}"
  "{pcores         | true | ⭐ 绑定到高性能核}"
  "{rtprio         | true | ⭐ 提高调度优先级到 -20}"
  "{csv            |      | ⭐ Debug CSV 输出前缀（给路径才落 CSV）}"

  "{record         | false | ⭐⭐ 录像到 records/（默认**不录**；录会占一个核做 MJPG 编码）}"
  "{pj             | false | ⭐ 是否发 PlotJuggler UDP}"
  "{debug-img      | false | ⭐ L3：启动就开存图（每 30 张 1 张，上限 500）}"
  "{debug-window   | false | ⭐ L3：启动就开可视化窗口（需 DISPLAY）}"
  "{tongji         | true | ⭐⭐ 同济兼容模式：true(默认)=完全同济行为；false=启用本项目优化}"
  "{strict-device  | false | ⭐ 严格设备模式：true=同济行为(设备不可用就抛异常)；false=回退CPU}";


using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  if (cli.get<bool>("pcores")) bind_to_p_cores();
  if (cli.get<bool>("rtprio")) elevate_priority();

  tools::Exiter exiter;
  tools::Plotter plotter;
  // ⭐⭐⭐ 「源代码以同济为准」：本开关集中控制**硬编码在源码里**的偏差
  //   默认 true = 完全同济行为。yaml 里的优化开关（弹道/过滤器/优先级）另算。
  {
    const bool tj = cli.get<bool>("tongji");
    tools::set_strict_device(cli.get<bool>("strict-device"));   // ⭐ W28
    auto_aim::Aimer::set_tongji_compat(tj);                       // E5：小陀螺判据用 x[8]（同济）还是 x[7]（修正）
    if (tj)
      tools::ExtendedKalmanFilter::use_tongji_nis_threshold();    // E3b：0.711（同济）
    else
      tools::ExtendedKalmanFilter::use_chi2_q95_nis_threshold();  // E3b：9.4877（χ²(4) 95% 分位）
    tools::logger()->info(
      "[hero] 同济兼容模式 = {}（--tongji=false 启用本项目优化）", tj ? "开" : "关");
  }

  // ⭐⭐ W48：录像**默认关**（录会占一个核做 MJPG 编码；要录传 --record）
  tools::Recorder recorder(30, cli.get<bool>("record"));

  // ⭐ W21：配置热重载（按键 `r`）
  //   ⚠️ 线程安全：`Tracker` 只被**主线程**用 → 主线程直接重载；
  //      `Planner` 被 **plan 线程**用 → 主线程只置标志，由 plan 线程自己去重载（免锁、无 race）
  tools::HotReloader hot_reloader(config_path);
  std::atomic<bool> reload_planner{false};


  // ⭐ 相机 + 下位机：真实硬件 / 录像回放，二选一（走同一套接口）
  const auto video_path = cli.get<std::string>("video");
  const bool use_video = !video_path.empty();

  std::unique_ptr<io::CameraBase> camera;
  std::unique_ptr<io::IBoard> board;

  if (use_video) {
    // 同名 .txt 回放云台四元数（没有则单位四元数）
    std::string pose_path = video_path;
    auto dot = pose_path.rfind('.');
    pose_path = (dot == std::string::npos) ? pose_path + ".txt" : pose_path.substr(0, dot) + ".txt";

    camera = std::make_unique<io::VideoCamera>(video_path, cli.get<double>("video-speed"));
    board = std::make_unique<io::ReplayBoard>(
      pose_path, io::GimbalMode::AUTO_AIM, static_cast<float>(cli.get<double>("bullet-speed")));
    tools::logger()->info("[hero] 录像回放模式: {}", video_path);
  } else {
    camera = std::make_unique<io::Camera>(config_path);
    board = std::make_unique<io::Gimbal>(config_path);   // ⭐ W31：直接是 IBoard
    tools::logger()->info("[hero] 真实硬件模式");
  }

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  // ⭐ W21：注册热重载组件（Tracker 属主线程）
  hot_reloader.on("ArmorFilter + Priority", [&tracker](const YAML::Node & y) { tracker.reload(y); });
  auto_aim::Planner planner(config_path);

  // ⭐ W16：Debug 数据面（SinkHub 可热插拔；空 hub → on_frame 就是空循环，零成本）
    // ⭐⭐ W49（方案 D）：**一行装配整个 Debug 体系**（原来手写 ~30 行）
    tools::DebugRuntime dbg(
      {.csv_prefix = cli.get<std::string>("csv"),
       .pj = cli.get<bool>("pj"),
       .img = cli.get<bool>("debug-img"),
       .window = cli.get<bool>("debug-window"),
       .name = "hero"},
      [&] {
        hot_reloader.reload_if_changed();   // 主线程：Tracker / 过滤器
        reload_planner = true;              // 交给 plan 线程：Planner
      });
    auto & hub = dbg.hub;                 // ⭐ 别名：保持下游代码一字不改
    auto & expense = dbg.expense;
    auto & hotkeys = dbg.hotkeys;
    bool & paused = dbg.paused;
    const auto csv_prefix = cli.get<std::string>("csv");

    tools::logger()->info(
      "[hero] 已注册 {} 个热重载组件（按键 r）", hot_reloader.size());
  tools::logger()->info("{}", hotkeys.help());

  // ⭐ 跨线程传 plan 结果（plan 在 plan_thread 里算，Debug 在主线程填）
  struct PlanSnapshot
  {
    std::mutex mtx;
    auto_aim::Plan plan;
    int64_t us = 0;
    bool valid = false;
  } psnap;

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  std::atomic<bool> quit = false;
  uint32_t frame_id = 0;

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      // ⭐ W21：热重载 Planner —— **在本线程执行**，避免与 plan() 竞争
      if (reload_planner.exchange(false)) {
        try {
          planner.reload(tools::load(config_path));
        } catch (const std::exception & e) {
          tools::logger()->error("[hero] Planner 热重载失败: {}", e.what());
        }
      }

      // ⭐ W11（J7）：原为 `if (!target_queue.empty()) { auto t = target_queue.front(); }`
      //   —— empty() 与 front() 两次加锁之间存在 TOCTOU；且 front() 返回 by-value 会拷整个 Target。
      //   try_peek 一次加锁完成「判空 + 拷贝取出」。
      std::optional<auto_aim::Target> target;
      if (mode == io::GimbalMode::AUTO_AIM && target_queue.try_peek(target)) {
        auto gs = board->state();
        const auto tp0 = std::chrono::steady_clock::now();
        auto plan = planner.plan(target, gs.bullet_speed);
        const auto plan_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - tp0).count();
        {
          std::lock_guard<std::mutex> lk(psnap.mtx);
          psnap.plan = plan;
          psnap.us = plan_us;
          psnap.valid = true;
        }

        board->send(
          plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
          plan.pitch_acc);

        std::this_thread::sleep_for(10ms);
      } else
        std::this_thread::sleep_for(200ms);
    }
  });

  while (!exiter.exit()) {
    mode = board->mode();

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::Gimbal::str_of(mode));
      last_mode = mode.load();
    }

    // ⭐ W18：热键（非阻塞，一帧一次）
    hotkeys.poll();
    if (paused) {
      std::this_thread::sleep_for(20ms);
      continue;
    }

    const auto t_frame0 = std::chrono::steady_clock::now();

    expense.begin("perceive");
    camera->read(img, t);

    // ⭐ W16 修复（W10 遗留 #3）：录像读完/相机掉线时的处理
    //   原来直接往下走 → YOLO 打印 "Empty img!" → 循环**满速空转**
    //   （录像模式下会瞬间写爆 CSV：实测 90 帧视频跑出 451 MB）
    if (img.empty()) {
      if (use_video) {
        tools::logger()->info("[hero] 录像播放完毕（共 {} 帧），退出", frame_id);
        break;
      }
      tools::logger()->warn("[hero] 相机空帧，等待 5ms 重试");
      std::this_thread::sleep_for(5ms);
      continue;
    }

    auto q = board->q(t);
    auto gs = board->state();
    recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);
    expense.end("perceive");

    std::vector<cv::Rect> dbg_boxes;   // ⭐ W19：L3 overlay 用（只有真要图时才画）

    // ⭐ W16：本帧调试快照
    auto_aim::FrameDebug fd;
    fd.frame_id = frame_id++;
    fd.t_perceive_us = expense.us("perceive");
    fd.mode = static_cast<uint8_t>(mode.load());
    fd.game_state = 0;                       // TODO: 接比赛状态（无敌/血量）后填
    fd.detector.armor_count = 0;
    fd.tracker.state = 0;

    /// 自瞄
    if (mode.load() == io::GimbalMode::AUTO_AIM) {
      expense.begin("detect");
      auto armors = yolo.detect(img);
      expense.end("detect");
      fd.detector.armor_count = static_cast<int>(armors.size());
      for (const auto & a : armors) dbg_boxes.push_back(a.box);
      fd.detector.t_infer_us = expense.us("detect");

      const int n_before = static_cast<int>(armors.size());
      expense.begin("track");
      auto targets = tracker.track(armors, t);
      expense.end("track");
      fd.tracker.t_track_us = expense.us("track");
      fd.tracker.priority_mode = static_cast<int>(tracker.priority_mode());
      fd.tracker.filtered_out = n_before - static_cast<int>(armors.size());
      fd.tracker.armor_count = static_cast<int>(armors.size());
      {  // tracker 状态字符串 -> 枚举
        const auto st = tracker.state();
        fd.tracker.state = (st == "tracking") ? 2 : (st == "temp_lost") ? 3
                          : (st == "detecting")            ? 1
                                                           : 0;
      }
      if (!targets.empty())
        target_queue.push(targets.front());
      else
        target_queue.push(std::nullopt);
    }

    /// 打符
    else if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
      // ⭐ W27：打符的 Debug 数据面（`BuffDebug` 原来定义了却没接线）
      const auto tb0 = std::chrono::steady_clock::now();
      buff_solver.set_R_gimbal2world(q);

      expense.begin("buff_detect");
      auto power_runes = buff_detector.detect(img);
      expense.end("buff_detect");

      expense.begin("buff_solve");
      buff_solver.solve(power_runes);
      expense.end("buff_solve");

      fd.buff.rune_type = (mode.load() == io::GimbalMode::SMALL_BUFF) ? 0 : 1;
      fd.buff.fanblade_count =
        power_runes ? static_cast<int>(power_runes->fanblades.size()) : 0;
      fd.buff.solved = power_runes.has_value();

      auto_aim::Plan buff_plan;
      if (mode.load() == io::GimbalMode::SMALL_BUFF) {
        buff_small_target.get_target(power_runes, t);
        fd.buff.spd = buff_small_target.spd;
        fd.buff.solved = !buff_small_target.is_unsolve();
        auto target_copy = buff_small_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
        buff_big_target.get_target(power_runes, t);
        fd.buff.spd = buff_big_target.spd;
        fd.buff.solved = !buff_big_target.is_unsolve();
        auto target_copy = buff_big_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      }
      fd.buff.t_us = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::steady_clock::now() - tb0).count();

      // 打符的 Plan 也是 auto_aim::Plan → 同样填 controller
      fd.controller.cmd_yaw = buff_plan.yaw;
      fd.controller.cmd_pitch = buff_plan.pitch;
      fd.controller.control = buff_plan.control;
      fd.controller.shoot = buff_plan.fire;
      fd.planner.t_fly = buff_plan.t_fly;
      fd.shooter.traj_err_at_fire = buff_plan.traj_err;
      fd.shooter.should_fire = buff_plan.fire;
      board->send(
        buff_plan.control, buff_plan.fire, buff_plan.yaw, buff_plan.yaw_vel, buff_plan.yaw_acc,
        buff_plan.pitch, buff_plan.pitch_vel, buff_plan.pitch_acc);

    } else
      board->send(false, false, 0, 0, 0, 0, 0, 0);

    // ⭐ W16：把 plan 线程的结果并入本帧快照 + 发布
    {
      std::lock_guard<std::mutex> lk(psnap.mtx);
      if (psnap.valid) {
        const auto & p = psnap.plan;
        fd.planner.t_plan_us = psnap.us;
        fd.planner.t_fly = p.t_fly;             // ⭐ W8 暴露的算法内部量
        fd.planner.overlap_ratio = p.overlap;
        fd.planner.solver_iters = p.yaw_iters;
        fd.planner.acc_max = p.acc_max;
        fd.shooter.traj_err_at_fire = p.traj_err;
        fd.shooter.fire_thresh = p.fire_thresh;
        fd.shooter.should_fire = p.fire;
        fd.controller.cmd_yaw = p.yaw;
        fd.controller.cmd_pitch = p.pitch;
        fd.controller.control = p.control;
      }
    }
    fd.t_frame_us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now() - t_frame0).count();
    hub.on_frame(fd);                          // ⭐ L0+L1 广播

#ifdef DEBUG_L3_ENABLE
    // ⭐ W19：L3 图像通道 —— **只有真有 L3 sink 时才构造 overlay**（否则零成本）
    if (hub.wants_image() && !img.empty()) {
      cv::Mat overlay = img.clone();
      for (const auto & b : dbg_boxes) cv::rectangle(overlay, b, {0, 255, 0}, 2);
      cv::putText(
        overlay, cv::format("f%u %s", fd.frame_id, tracker.state().c_str()), {12, 40},
        cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 255}, 2);
      hub.on_image("aim", overlay, fd.t_frame_us);
    }
#endif
    hub.on_series("planner.t_fly", fd.frame_id, fd.planner.t_fly);
    hub.on_series("planner.overlap", fd.frame_id, fd.planner.overlap_ratio);
    hub.on_series("planner.acc_max", fd.frame_id, fd.planner.acc_max);

    expense.next_frame();
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  if (!csv_prefix.empty())
    tools::logger()->info("[hero] Debug CSV 已写出: {}_frames.csv / {}_series.csv",
                          csv_prefix, csv_prefix);
  board->send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}