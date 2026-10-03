#include <algorithm>
#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "core/auto_aim/planner/mpc.hpp"
#include "utils/concurrency/exiter.hpp"
#include "utils/log/logger.hpp"
#include "utils/math/math_tools.hpp"
// ⭐ W8：Debug 数据面
#include "core/debug_node.hpp"
#include "utils/debug/csv_sink.hpp"
#include "utils/debug/debug_sink.hpp"
#include "utils/debug/expense.hpp"
#include "utils/debug/plotjuggler_sink.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |     | 输出命令行参数说明    }"
  "{d              | 3.0 | Target距离(m)       }"
  "{w              | 5.0 | Target角速度(rad/s) }"
  "{@config-path   |     | yaml配置文件路径     }"
  "{csv c          |      | ⭐ CSV 输出前缀（不传则不落 CSV）}"
  "{pj             | true | ⭐ 是否发 PlotJuggler UDP}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  auto d = cli.get<double>("d");
  auto w = cli.get<double>("w");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  // ⭐ W8：新 Debug 数据面（SinkHub 可热插拔）
  tools::SinkHub hub;
  auto csv_prefix = cli.get<std::string>("csv");
  if (!csv_prefix.empty()) hub.add(std::make_shared<tools::CsvSink>(csv_prefix));
  hub.add(std::make_shared<tools::PlotJugglerSink>("127.0.0.1", 9870, cli.get<bool>("pj")));
  tools::Expense expense;

  auto_aim::Planner planner(config_path);
  auto_aim::Target target(d, w, 0.2, 0.1);

  auto t0 = std::chrono::steady_clock::now();
  uint32_t frame_id = 0;
  int solver_iters = 0;

  while (!exiter.exit()) {
    target.predict(0.01);

    expense.begin("plan");
    auto plan = planner.plan(target, 22);
    expense.end("plan");

    // ⭐⭐⭐ W101（原 F6）：**这里原来是"新旧路径对照"** —— 旧路径是
    //   `tools::Plotter().plot(json)`（同济的 plotter，无 timestamp、每帧建 socket、
    //   且**绕过 SinkHub**）。⚠️ 对照的目的已经达到（新路径 `PlotJugglerSink` 胜出），
    //   而旧 plotter **已删除** ⇒ 这段 json 组装 + 调用一并去掉。
    //   ⭐ 新路径在下面：填 `FrameDebug` → `hub.on_frame(fd)` → 各 sink。
    const auto t_frame_us =
      static_cast<int64_t>(tools::delta_time(std::chrono::steady_clock::now(), t0) * 1e6);

    // ⭐ W8：填 FrameDebug（L0 耗时 + L1 快照，~ns 级成本）+ 发布
    auto_aim::FrameDebug fd;
    fd.frame_id = frame_id++;
    fd.t_frame_us = t_frame_us;
    // ⭐⭐ W98/W101：**路由映射收进 `Plan::fill_debug()`**（原来这里手写 9 行）
    plan.fill_debug(fd.planner, fd.shooter, fd.controller);
    fd.planner.t_plan_us = expense.us("plan");
    fd.tracker.filtered_out = 0;
    fd.tracker.priority_mode = 3;
    hub.on_frame(fd);
    // ⭐ L2 曲线
    hub.on_series("planner.yaw", fd.t_frame_us, plan.yaw);
    hub.on_series("planner.yaw_vel", fd.t_frame_us, plan.yaw_vel);
    hub.on_series("planner.yaw_acc", fd.t_frame_us, plan.yaw_acc);
    hub.on_series("planner.acc_max", fd.t_frame_us, fd.planner.acc_max);
    hub.on_series("planner.traj_err", fd.t_frame_us, plan.traj_err);
    hub.on_series("planner.overlap", fd.t_frame_us, plan.overlap);
    hub.on_series("planner.t_fly", fd.t_frame_us, plan.t_fly);

    std::this_thread::sleep_for(10ms);
  }

  return 0;
}