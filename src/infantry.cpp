#include <chrono>
#include <opencv2/opencv.hpp>
#include <memory>
#include <mutex>
#include <thread>

#include "io/camera/camera.hpp"
#include "io/camera/video.hpp"        // ⭐ 录像回放相机
#include "io/board/board.hpp"         // ⭐ IBoard 统一接口
#include "io/board/gimbal/gimbal.hpp"
#include "io/board/make_board.hpp"   // ⭐ W54：下位机自动感应   // ⭐ W31：io::Gimbal 已直接实现 IBoard
#include "io/board/replay.hpp"        // ⭐ 录像回放的「下位机」
#include "drivers/dm_imu/dm_imu.hpp"
// ⭐ 显式补上：同济 standard_mpc.cpp **没有** include yolo.hpp，
//    auto_aim::YOLO 是靠 mt_detector.hpp **传递**拿到的（隐性依赖，同 C23 一类）
#include "core/auto_aim/detector/detector_slot.hpp"   // ⭐ W101（原 B2）
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
#include "utils/log/log_filter.hpp"
#include "utils/config/stage_gate.hpp"   // ⭐ W100
#include "utils/config/print_config.hpp"   // ⭐ W100
#include "utils/config/tongji_flags.hpp"   // ⭐ W100：tongji 拆槽位
#include "utils/log/logger.hpp"
#include "utils/math/math_tools.hpp"
#include "utils/debug/recorder.hpp"
// ⭐ W16：Debug 数据面（W8 建好，这次接进主程序）
#include "core/auto_aim/detector/det_stats.hpp"
#include "utils/system/disk_guard.hpp"
#include "utils/system/paths.hpp"   // ⭐ W87   // ⭐ W83
#include "utils/system/host_info.hpp"
#include "utils/system/thread_tuning.hpp"   // ⭐ W82   // ⭐ W81：本机核数 + 建议
#include "core/auto_aim/target/target_debug_fill.hpp"   // ⭐ W70   // ⭐ W63
#include "core/debug_node.hpp"
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


const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }"
  "{print-config   | false | ⭐⭐ 打印最终生效配置后退出（不启动相机/板卡）}"
  "{video v        | | ⭐ 录像路径（.avi）；给了就走录像回放，无需相机/下位机}"
  "{video-speed    | 1.0 | ⭐ 录像播放速率（1.0=实时；调大=快放；⭐ 0=不节流全速跑批）}"
  "{bullet-speed   | 22.0 | 录像模式下的弹速（下位机不可用时）}"
  "{dump-camera-params | false | ⭐ 只打印相机常用参数的当前值（用于把 MVS 里的好值抄进 yaml 的 camera_params）}"
  "{camera-config  | | ⭐ 覆盖相机配置路径（默认用兵种 yaml 的 camera_config 键）}"
  "{csv            |      | ⭐ Debug CSV 输出前缀（给路径才落 CSV）}"

  "{record         | false | ⭐⭐ 录像到 output/video/（默认**不录**；录会占一个核做 MJPG 编码）}"
  "{pj             | false | ⭐ 是否发 PlotJuggler UDP}"
  "{pj-host        | 127.0.0.1 | ⭐ PlotJuggler 目标 IP（跨机器时填对方 IP）}"
  "{pj-port        | 9870 | ⭐ PlotJuggler 目标端口}"
  "{debug-img      | false | ⭐ L3：启动就开存图（每 30 张 1 张，上限 500）}"
  "{stop-after     | | ⭐⭐ 算法独立测试：跑到该阶段就停（perceive/detect/track/buff-detect/buff-solve/plan；空=全跑）}"
  "{debug-only     | | ⭐⭐ 一条命令配齐「只看这一步」= --stop-after + --log-only（perceive/detect/track/buff-detect/buff-solve/plan）}"
  "{debug-window   | false | ⭐ L3：启动就开可视化窗口（需 DISPLAY）}"
  "{tongji         | true | ⭐⭐ 同济兼容模式：true(默认)=完全同济行为；false=启用本项目优化}"
  "{nis-thresh     | | ⭐ 单独覆盖 NIS 失败阈值：tongji(0.711) / chi2(9.4877)；空=跟随 --tongji}"
  "{yaw-rate-src   | | ⭐ 单独覆盖小陀螺判据用的 EKF 分量：x8(同济) / x7(修正)；空=跟随 --tongji}"
  "{force-mode     | -1 | ⭐ 录像模式强制档位：0=IDLE 1=自瞄 2=小符 3=大符（便于零硬件验证打符）}"
  "{strict-device  | false | ⭐ 严格设备模式：true=同济行为(设备不可用就抛异常)；false=回退CPU}"
  "{no-board       | false | ⭐⭐ 强制虚拟下位机（不碰串口；只有摄像头时用）}"
  "{strict-board   | false | ⭐⭐ 串口不存在就失败退出（同济行为）；默认自动降级虚拟板}"
  "{det-stats      | true | ⭐ 逐帧打印检测统计（候选→各步过滤）；false 硬关}"
  "{log-keep-days  | 30 | ⭐ 日志保留天数（超期自动清理 output/logs/；0=不清理）}"
  "{log-off        | | ⭐ 按模块静音日志（运行期，不重编；⚠️ 有 89ns/次代价，长期请用 utils/log/debug_config.hpp 的编译期开关）}"
  "{log-only       | | ⭐ 只打印这些模块（运行期；同 --log-off 的说明）}";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");

  // ⭐⭐ W100（原 D2）：`--print-config` 打完就退，**不碰相机/板卡**
  if (cli.get<bool>("print-config")) {
    tools::print_effective_config(std::cout, cli, config_path, "infantry");
    return 0;
  }


  // ⭐⭐ W100（原 A2）：**算法独立测试模式** —— 跑到该阶段就停，截断后面的链路

  //   ⭐ 用途：单独验证某一环（如"纯检测率"不受跟踪过滤影响）

  // ⭐⭐ W101（原 A3）：`--debug-only=<阶段>` = `--stop-after` + `--log-only` 一次配齐
  //   ⚠️ 优先级：**显式参数优先**（`--debug-only` 只在对应参数没给时才填）
  const auto debug_only = tools::parse_debug_only(cli.get<std::string>("debug-only"));
  const auto stop_after = debug_only.active && cli.get<std::string>("stop-after").empty()
                            ? debug_only.stop
                            : tools::parse_stop_after(cli.get<std::string>("stop-after"));
  if (debug_only.active)
    tools::logger()->info("[{}] --debug-only ⇒ 截断到 {} + 只看相关模块", "infantry",
                          tools::stage_name(stop_after));

  if (tools::stage_truncated(stop_after))

    tools::logger()->warn(

      "[infantry] ⚠️ 算法独立测试：--stop-after={} ⇒ **链路被截断**，后面阶段不跑",

      tools::stage_name(stop_after));

  // ⭐⭐⭐ W95：**按模块过滤日志必须尽早设置** —— 否则启动期日志
  //   （如 `[infantry] 同济兼容模式` / `[VideoCamera]` / `[ReplayBoard]`）
  //   会在过滤生效前就打出来（实测踩过）。
  {
    const auto log_off = cli.get<std::string>("log-off");

    // ⭐ W101（原 A3）：`--debug-only` 时若没显式给 `--log-only`，用它填

    const std::string log_only_raw = cli.get<std::string>("log-only").empty()

                                         && debug_only.active

                                       ? debug_only.log_only

                                       : cli.get<std::string>("log-only");

    const auto log_only = log_only_raw;
    if (!log_off.empty())  tools::set_log_modules_off(tools::parse_module_list(log_off));
    if (!log_only.empty()) tools::set_log_modules_only(tools::parse_module_list(log_only));

    // ⭐ 打一条状态（用 logger，能同时验证过滤已生效）

    if (!log_off.empty() || !log_only.empty())

      tools::logger()->info("[log] {}", tools::log_filter_status());
  }
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  // ⭐⭐⭐ 「源代码以同济为准」：本开关集中控制**硬编码在源码里**的偏差
  //   默认 true = 完全同济行为。yaml 里的优化开关（弹道/过滤器/优先级）另算。
    // ⭐⭐⭐ W100：`--tongji` 拆成独立槽位（原 B3/B4）——
  //   单项显式指定 > 总开关 > 内置默认（同济行为）。见 `utils/config/tongji_flags.hpp`
  {
    tools::set_strict_device(cli.get<bool>("strict-device"));
    auto_aim::set_det_stats_enabled(cli.get<bool>("det-stats"));   // ⭐ W63
    const auto flags = tools::resolve_tongji(
      cli.get<bool>("tongji"), cli.get<std::string>("nis-thresh"),
      cli.get<std::string>("yaw-rate-src"));
    flags.apply();
    tools::logger()->info("[infantry] {}", flags.describe());
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
    const int fm = cli.get<int>("force-mode");
    const auto replay_mode = (fm >= 0 && fm <= 3) ? static_cast<io::GimbalMode>(fm)
                                                  : io::GimbalMode::AUTO_AIM;
    board = std::make_unique<io::ReplayBoard>(
      pose_path, replay_mode, static_cast<float>(cli.get<double>("bullet-speed")));
    tools::logger()->info("[infantry] 录像回放模式: {}", video_path);
  } else {
    camera = std::make_unique<io::Camera>(config_path, cli.get<bool>("dump-camera-params"), cli.get<std::string>("camera-config"));
    // ⭐⭐ W54：**下位机自动感应** —— 串口存在用真板子；
    //   不存在就**降级虚拟板**（只有摄像头时也能跑），而不是 exit(1)。
    //   要恢复同济的"没下位机就失败" → `--strict-board`
    board = io::make_board(
      config_path, cli.get<bool>("no-board"), cli.get<bool>("strict-board"),
      cli.get<double>("bullet-speed"), -1, "infantry");
  }

  auto_aim::DetectorSlot yolo(config_path, true);   // ⭐ W101（原 B2）：统一槽位
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  // ⭐ W21：注册热重载组件（Tracker 属主线程）
  hot_reloader.on("ArmorFilter + Priority", [&tracker](const YAML::Node & y) { tracker.reload(y); });
  auto_aim::Planner planner(config_path);

  // ⭐ W16：Debug 数据面（SinkHub 可热插拔；空 hub → on_frame 就是空循环，零成本）
    // ⭐⭐ W49（方案 D）：**一行装配整个 Debug 体系**
    //   原来这里手写 ~30 行（hub/expense/hotkeys/factories/L3/绑定/帮助）
    //   → 收进 `tools::DebugRuntime`，4 个兵种共用，加新 sink 只改一处
    // ⭐⭐ W81：启动时打印「本机核数 + 建议配置」（核少时提示关 sink）
    // ⭐⭐ W83：录像磁盘守卫（**启动时一次**：配额清理 + 剩余空间 + 低空间警告）
    tools::guard_on_startup(cli.get<int>("log-keep-days"), tools::paths::video());   // ⭐ W84

    tools::DebugRuntime dbg(
      {.csv_prefix = cli.get<std::string>("csv"),
       .pj = cli.get<bool>("pj"),
      .pj_host = cli.get<std::string>("pj-host"),
      .pj_port = static_cast<uint16_t>(cli.get<int>("pj-port")),
       .img = cli.get<bool>("debug-img"),
       .window = cli.get<bool>("debug-window"),
       .name = "infantry"},
      [&] {
        hot_reloader.reload_if_changed();   // 主线程：Tracker / 过滤器
        reload_planner = true;              // 交给 plan 线程：Planner
      });
    auto & hub = dbg.hub;                 // ⭐ 别名：保持下游代码一字不改
    auto & expense = dbg.expense;
    auto & hotkeys = dbg.hotkeys;
    bool & paused = dbg.paused;
    const auto csv_prefix = cli.get<std::string>("csv");

  // ⭐ 跨线程传 plan 结果（plan 在 plan_thread 里算，Debug 在主线程填）
  struct PlanSnapshot
  {
    std::mutex mtx;
    auto_aim::Plan plan;
    int64_t us = 0;       // planner.plan() 耗时（本线程测）
    int64_t ctl_us = 0;   // ⭐ W73：board->send() 耗时（真机上=串口写）
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
          tools::logger()->error("[infantry] Planner 热重载失败: {}", e.what());
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

        // ⭐⭐ W73：测 `board->send()` 的真实耗时（真机上就是串口写）——
        //   原来 `ctl_t_us` 没地方取，我先填 0（会变成死列）。现在在这条线程里量。
        const auto tc0 = std::chrono::steady_clock::now();
        board->send(
          plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
          plan.pitch_acc);
        const auto ctl_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - tc0).count();
        {
          std::lock_guard<std::mutex> lk(psnap.mtx);
          psnap.ctl_us = ctl_us;
        }

        std::this_thread::sleep_for(10ms);
      } else
        std::this_thread::sleep_for(200ms);
    }
  });

    // ⭐⭐ W55：空帧计数（日志节流用）
    int64_t empty_frames = 0;

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

    // W86: time the camera wait separately -- on real hardware this is a
    //   blocking wait (~28 ms @30fps), NOT CPU work. Keeping it inside
    //   "perceive" made the frame-budget report read ~280%.
    expense.begin("cam_wait");
    camera->read(img, t);
    expense.end("cam_wait");

    expense.begin("perceive");

    // ⭐ W16 修复（W10 遗留 #3）：录像读完/相机掉线时的处理
    //   原来直接往下走 → YOLO 打印 "Empty img!" → 循环**满速空转**
    //   （录像模式下会瞬间写爆 CSV：实测 90 帧视频跑出 451 MB）
    if (img.empty()) {
      if (use_video) {
        tools::logger()->info("[infantry] 录像播放完毕（共 {} 帧），退出", frame_id);
        break;
      }
      // ⭐⭐ W55：**节流** —— 相机彻底掉线时每 5ms 走到这里（200 次/秒），
      //   不限流就是 200 行/秒刷屏。前 3 次 + 之后每 200 次（约 1 秒）报一次。
      ++empty_frames;
      if (empty_frames <= 3 || empty_frames % 200 == 0)
        tools::logger()->warn(
          "[infantry] 相机空帧 ×{}（等待 5ms 重试；Ctrl-C 可退出）", empty_frames);
      std::this_thread::sleep_for(5ms);
      continue;
    }

    auto q = board->q(t);
    auto gs = board->state();
    recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);
    expense.end("perceive");

    // ⭐⭐⭐ W112：**L3 overlay 的绘制素材** —— 原来只存 `cv::Rect`（外接正矩形），
    //   ⚠️ **丢了四点轮廓和标签** ⇒ `aim` 窗口比同济的 `detection` 窗口信息少。
    //   ⭐ 现在存**四点 + 中心 + 标签**，对齐同济的 `draw_detections`。
    //   ⚠️ **只在真要图时才填**（`hub.wants_image()`）⇒ 没人看图时**零开销**。
    struct DbgArmor
    {
      std::vector<cv::Point2f> points;   // ⭐ 模型输出的四点（连起来是装甲板四边形）
      cv::Point2f center;                // 标签画在哪
      std::string label;                 // ⭐ "0.95 blue 3 big"
    };
    std::vector<DbgArmor> dbg_armors;

    // ⭐⭐⭐ W113：**EKF 预测 + 瞄准点**（照搬旧版赫兹，新版重构时没搬）
    //   ⚠️ 为什么必须"带出来"：`targets`（442 行）和 `psnap.plan`（515 行）
    //     都在**更内层的作用域**里，而 overlay 在 551 行 ⇒ 只能先把【已投回像素的点】
    //     存下来。⭐ 好处：投影只在真要图时算（`wants_image()` 门控）。
    std::vector<std::vector<cv::Point2f>> dbg_pred;   // ⭐ 绿：EKF 预测的装甲板四点
    std::vector<cv::Point2f> dbg_aim;                 // ⭐⭐ 红：瞄准点四点
    // ⭐ 瞄准点投影要用【目标的类型/名字】（决定装甲板尺寸）⇒ 从 track 段带出来
    auto_aim::ArmorType dbg_tgt_type = auto_aim::ArmorType::small;
    auto_aim::ArmorName dbg_tgt_name = auto_aim::ArmorName::not_armor;

    // ⭐ W16：本帧调试快照
    auto_aim::FrameDebug fd;
    fd.frame_id = frame_id++;
    fd.t_cam_wait_us = expense.us("cam_wait");   // W86: blocking wait, not CPU
    fd.t_perceive_us = expense.us("perceive");
    fd.mode = static_cast<uint8_t>(mode.load());
    // ⭐⭐⭐ W101（原 G1）：**这个字段【暂时无法填】，原因具体如下**（不是"忘了"）：
    //   · `game_state` 语义 = 比赛阶段 / 血量 / 无敌 等**裁判系统**信息；
    //   · 本项目的两个数据源都**不提供**：
    //       - 下位机（`io/board/`）：只有 `bullet_speed` / `bullet_count` / `mode`
    //       - ROS2 桥（`io/ros2/`）：只有 `publish(target_pos)` /
    //         `subscribe_enemy_status()`（无敌 id）/ `subscribe_autoaim_target()`（集火 id）
    //   · ⭐ **部分替代已经有了**：sentry 的 `fd.tracker.invincible_count` 是**真实值**
    //     （来自 `subscribe_enemy_status()`）—— 只放开那个，不编这个。
    // ⇒ **要填它必须先有裁判系统接入**（ROS2 自定义 msg 或串口协议）。在那之前：
    //   ⚠️ **保持 0 并如实说明**，比填一个"看起来有值"的东西（会被当真的用）安全。
    fd.game_state = 0;
    fd.detector.armor_count = 0;
    fd.tracker.state = 0;

    /// 自瞄
    if (mode.load() == io::GimbalMode::AUTO_AIM) {
      // ⭐⭐⭐ W100（原 A2）：**阶段门** —— `--stop-after` 截断链路
      //   ⚠️ 为什么是"截断前缀"而不是"跳过中间某步"：跟踪没有检测输出就没法跑
      //      （数据流上"跳过"不成立），能截的只有前缀。
      const bool do_detect = tools::stage_ok(stop_after, tools::Stage::Detect);
      const bool do_track  = tools::stage_ok(stop_after, tools::Stage::Track);
      const bool do_plan   = tools::stage_ok(stop_after, tools::Stage::Plan);

      std::list<auto_aim::Armor> armors;    // ⭐ 提到外层：两个阶段共用
      if (do_detect) {
        expense.begin("detect");
        // ⭐⭐ W98：**结果 + 调试快照一起返回** —— 不再"手写填 + 读全局旁路"
        auto det = yolo.detect(img);
        armors = std::move(det.armors);
        expense.end("detect");
        fd.detector = det.dbg;                               // ⭐ 一行，不可能忘
        fd.detector.t_infer_us = expense.us("detect");
        // ⭐⭐ W112：**只在真要图时才填**（原来无条件填 `Rect`，白花）
        if (hub.wants_image()) {
          for (const auto & a : armors) {
            const int ci = static_cast<int>(a.color), ni = static_cast<int>(a.name),
                      ti = static_cast<int>(a.type);
            dbg_armors.push_back(
              {a.points, a.center,
               cv::format("%.2f %s %s %s", a.confidence,
                          (ci >= 0 && ci < (int)auto_aim::COLORS.size()) ? auto_aim::COLORS[ci].c_str() : "?",
                          (ni >= 0 && ni < (int)auto_aim::ARMOR_NAMES.size()) ? auto_aim::ARMOR_NAMES[ni].c_str() : "?",
                          (ti >= 0 && ti < (int)auto_aim::ARMOR_TYPES.size()) ? auto_aim::ARMOR_TYPES[ti].c_str() : "?")});
          }
        }
      }

      std::list<auto_aim::Target> targets;  // ⭐ 同上
      if (do_track) {
        expense.begin("track");
        auto trk = tracker.track(armors, t);   // ⭐⭐ W98：目标 + 调试快照一起返回
        targets = std::move(trk.targets);
        expense.end("track");
        fd.tracker = trk.dbg;                  // ⭐ 一行替代原来的 6 处手写
        fd.tracker.t_track_us = expense.us("track");
        // ⭐⭐ W71：`sol_*` 四列（原来永远是 0）—— Solver 在 Tracker 内部被调用
        fd.solver = trk.solver_dbg;   // ⭐ W101（原 F8）：随 TrackerResult 带出
        // ⭐⭐ W70：填 `tgt_*`（原来 `fd.target.*` **从没被赋值** → CSV 里 13 列永远 0）
        //   EKF 状态布局（见 target.cpp）：x vx y vy z vz a w r l h
        if (!targets.empty()) {
          // ⭐⭐⭐ W113：**EKF 预测点 + 记录目标类型/名字**（给后面的瞄准点投影用）
          //   ⭐ 成本：只在真要图时做（`wants_image()`）⇒ 没人看图时零开销。
          if (hub.wants_image()) {
            const auto & tg = targets.front();
            dbg_tgt_type = tg.armor_type;
            dbg_tgt_name = tg.name;
            for (const auto & xyza : tg.armor_xyza_list()) {
              dbg_pred.push_back(solver.reproject_armor(
                xyza.head(3), xyza[3], tg.armor_type, tg.name));   // ⭐ 3D → 像素
            }
          }
          auto_aim::fill_target_debug(fd.target, targets.front(), fd.solver.t_solve_us);
          // ⭐⭐ W72：`tgt_invincible` + `sht_blocked_inv`（原来都是 0）
          //   无敌在 Tracker 的 filter 层判定 → 这里查掩码是否含该目标的兵种
          fd.target.invincible = tracker.invincible().has(targets.front().name);
          fd.shooter.blocked_by_invincible = fd.target.invincible;
        }
      }

      // ⭐ 只有跑到 plan 阶段才喂目标给规划线程；否则喂 nullopt ⇒ **不发云台指令**
      if (do_plan && !targets.empty())
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

      // ⭐ W98：rune_type/fanblade_count/solved 由 helper 填

      if (power_runes) power_runes->fill_debug(fd.buff);

      auto_aim::Plan buff_plan;
      if (mode.load() == io::GimbalMode::SMALL_BUFF) {
        buff_small_target.get_target(power_runes, t);
        buff_small_target.fill_debug(fd.buff);   // ⭐ W98：spd + solved
        auto target_copy = buff_small_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
        buff_big_target.get_target(power_runes, t);
        buff_big_target.fill_debug(fd.buff);   // ⭐ W98：spd + solved
        auto target_copy = buff_big_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      }
      fd.buff.t_us = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::steady_clock::now() - tb0).count();

      // ⭐ W98：打符的 Plan 也是 auto_aim::Plan → 复用同一个路由

      buff_plan.fill_debug(fd.planner, fd.shooter, fd.controller);
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
        // ⭐⭐ W98：**路由映射收进 `Plan::fill_debug()`**（原来四兵种各手写 10 行）
        p.fill_debug(fd.planner, fd.shooter, fd.controller);
        // ⭐⭐⭐ W113：**瞄准点投回像素**（旧版赫兹的"红圈"就是它）
        //   `Plan::debug_xyza` = 瞄准点的 (x,y,z,yaw)（`mpc.cpp:209` 被填）
        //   ⚠️ 只在 [真要图] 且 [有目标] 时算 —— 每帧几次 4×4 矩阵乘，可忽略。
        if (hub.wants_image() && dbg_tgt_name != auto_aim::ArmorName::not_armor &&
            p.debug_xyza.head(3).norm() > 1e-6) {
          dbg_aim = solver.reproject_armor(
            p.debug_xyza.head(3), p.debug_xyza[3], dbg_tgt_type, dbg_tgt_name);
        }
        fd.planner.t_plan_us = psnap.us;
        fd.controller.t_ctrl_us = psnap.ctl_us;   // ⭐ W73：board->send() 真实耗时
      }
    }
    // ⭐⭐ W71：`t_decide_us`（原来永远是 0）= 从 perceive 结束到本帧决策完成的耗时
    fd.t_decide_us = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::steady_clock::now() - t_frame0).count() - fd.t_perceive_us;
    if (fd.t_decide_us < 0) fd.t_decide_us = 0;
    // ⭐⭐ W71：`t_since_last_fire_us` —— 距上次开火的间隔（0 = 本帧刚开火）
    {
      static std::chrono::steady_clock::time_point t_last_fire{};
      // ⭐⭐ W72：`sht_blocked_filter` = 「有控制 + 有目标，但没开火」= 判据没过
      //   （infantry/hero/sentry 走 `Plan.fire` 的 MPC 判据；uav 走 `Shooter`）
      fd.shooter.blocked_by_filter =
        fd.controller.control && (fd.tracker.armor_count > 0) && !fd.shooter.should_fire;
      if (fd.shooter.should_fire) t_last_fire = std::chrono::steady_clock::now();
      fd.shooter.t_since_last_fire_us =
        t_last_fire.time_since_epoch().count() == 0
          ? 0
          : std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - t_last_fire).count();
    }
    fd.t_frame_us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now() - t_frame0).count();
    hub.on_frame(fd);                          // ⭐ L0+L1 广播

    // ⭐⭐⭐ W61：**不再用 `#ifdef`** —— 原来这段被 `DEBUG_L3_ENABLE` 包着，
    //   Release 下**整个不编译** → 按 1 能开出窗口但**永远黑屏**（实测踩到）。
    //   现在总是编入，**纯运行期门控**：没人要图 → `wants_image()` 返回 false
    //   → **连 `clone()` 都不做**（实测 `wants_image()` ≈ 24 ns/帧 = 0.00024% 预算）。
    // ⭐ W19：L3 图像通道 —— **只有真有 L3 sink 时才构造 overlay**（否则零成本）
    if (hub.wants_image() && !img.empty()) {
      cv::Mat overlay = img.clone();
      // ⭐⭐⭐ W112：**画四点轮廓 + 标签**（对齐同济 `detection` 窗口）
      //   ⚠️ 原来画的是 `cv::rectangle(a.box)` = 四点的**外接正矩形**
      //     ⇒ **装甲板有倾角时，正矩形比实际四边形"胖一圈"**，且完全没有标签。
      //   ⭐ `draw_points` 内部是 `cv::drawContours` ⇒ **把四点连成闭合四边形**（可斜）。
      // ⭐⭐⭐ W115：**三种框的配色定稿**（用户要求）
      //   | 画什么 | 颜色 | 粗细 |
      //   |---|---|---|
      //   | ⭐ **检测四点**（这一帧看到了什么） | 绿 `{0,255,0}` | **粗（3）** |
      //   | ⭐ **EKF 预测四点**（EKF 认为目标在哪） | ⭐ **淡蓝 `{255,200,0}`** | **粗（3）** |
      //   | ⭐ **瞄准框**（弹丸飞 `t_fly` 后的位置） | 红 `{0,0,255}` | **粗（3）** |
      //   · ⚠️ **不要十字圈**（原来的 `draw_aim_point`）—— 红框表达"瞄哪"已经够，
      //     再加圆圈/十字会**盖住装甲板**（用户要求去掉）。
      //   · ⭐ **瞄准点另给一个小实心圆点**（红框中心）—— 一眼看出"瞄的位置"。
      const cv::Scalar kGreen{0, 255, 0};        // 检测
      const cv::Scalar kLightBlue{255, 200, 0};  // ⭐ EKF 预测（BGR：蓝为主 + 一点绿 ⇒ 淡蓝）
      const cv::Scalar kRed{0, 0, 255};          // 瞄准

      for (const auto & d : dbg_armors) {
        if (d.points.size() >= 2) tools::draw_points(overlay, d.points, kGreen, 3);
        tools::draw_text(overlay, d.label, d.center, kGreen, 0.6, 1);
      }
      for (const auto & pts : dbg_pred) {
        if (pts.size() >= 2) tools::draw_points(overlay, pts, kLightBlue, 3);   // ⭐ 淡蓝 + 粗
      }
      if (dbg_aim.size() >= 2) {
        tools::draw_points(overlay, dbg_aim, kRed, 3);                          // ⭐ 红框（不要圈/十字）
        cv::Point2f c(0, 0);
        for (const auto & p : dbg_aim) c += p;
        c.x /= static_cast<float>(dbg_aim.size());
        c.y /= static_cast<float>(dbg_aim.size());
        cv::circle(overlay, cv::Point(static_cast<int>(c.x), static_cast<int>(c.y)), 6, kRed, -1);
        cv::circle(overlay, cv::Point(static_cast<int>(c.x), static_cast<int>(c.y)), 10, kRed, 2);
      }
      cv::putText(
        overlay, cv::format("f%u %s", fd.frame_id, tracker.state().c_str()), {12, 40},
        cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 255}, 2);
      // ⭐⭐⭐ W116：**这里【不】resize** —— 缩放交给各个 sink 自己决定
      //   ⚠️ W115 我在这里加了一句 `resize 0.5`，**和 `WindowSink` 自带的 `scale_=0.5` 叠加**
      //     ⇒ 窗口里实际显示 **360×270**，再被 `WINDOW_NORMAL` 放大回去 ⇒ **插值 ⇒ 糊**。
      //   ⭐ 分工：**发布端给全尺寸原图**，由 sink 各取所需：
      //     · `WindowSink`：`scale_`（默认 0.5）⇒ 720×540，配 `WINDOW_AUTOSIZE` ⇒ **1:1 清晰**
      //     · `ImageSink` ：存**全尺寸**（调参时能放大看细节）
      //     · PlotJuggler / CSV：不涉及图
      hub.on_image("aim", overlay, fd.t_frame_us);
    }
    hub.on_series("planner.t_fly", fd.frame_id, fd.planner.t_fly);
    hub.on_series("planner.overlap", fd.frame_id, fd.planner.overlap_ratio);
    hub.on_series("planner.acc_max", fd.frame_id, fd.planner.acc_max);

    expense.next_frame();
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  if (!csv_prefix.empty())
    tools::logger()->info("[infantry] Debug CSV 已写出: {}_frames.csv / {}_series.csv",
                          csv_prefix, csv_prefix);
  board->send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}