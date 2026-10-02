#include "recorder.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <string>

#include "utils/log/logger.hpp"
#include "utils/system/thread_tuning.hpp"   // ⭐ W82
#include "utils/math/math_tools.hpp"

namespace tools
{

Recorder::Recorder(double fps, bool enabled, size_t queue_cap)
: fps_(fps),
  enabled_(enabled),
  // ⭐⭐ W48 ③：队列容量 1 → 可配（默认 8）。
  //   ⚠️ 满时**丢新帧**（`PopWhenFull=false`），但**不再静默**（见下）。
  queue_(queue_cap < 2 ? 2 : queue_cap,
         // ⭐⭐ W48 ②：`full_handler` —— 原来默认是空 lambda → **静默丢帧**
         [this] {
           const auto n = ++dropped_;
           if (!drop_warned_.exchange(true)) {
             tools::logger()->warn(
               "[Recorder] ⚠️ **队列已满，开始丢帧**（worker 的 MJPG 编码跟不上 {} fps）。"
               "录出来的视频会**跳帧**。丢帧数会在退出时报告。",
               fps_);
           }
           (void)n;
         })
{
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;

  auto folder_path = "records";
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  text_path_ = fmt::format("{}/{}.txt", folder_path, file_name);
  video_path_ = fmt::format("{}/{}.avi", folder_path, file_name);

  std::filesystem::create_directory(folder_path);

  // ⭐⭐ W48 ①：**只有显式开启才真的录**
  //   原来构造函数定好路径、主循环无条件 record() → 跑任何程序都在录
  //   （实测跑几十次测试就攒了 48 个文件）
  if (enabled_) {
    tools::logger()->info(
      "[Recorder] ⭐ 录像已开启 → {}  /  {}（{} fps，队列容量 {}{}）", video_path_, text_path_,
      fps_, queue_.capacity(), "");
  } else {
    tools::logger()->info(
      "[Recorder] 录像**未开启**（要录请传 `--record` 或热键开启）—— 不会产生 records/ 文件");
  }
}

Recorder::~Recorder() { close(); }

void Recorder::close()
{
  // ⭐ 幂等：析构会再调一次
  if (closed_.exchange(true)) return;

  if (!init_) {
    // 未初始化（没录到任何帧）→ 删掉空文件，并说明
    if (!enabled_) return;
    std::error_code ec;
    std::filesystem::remove(text_path_, ec);
    std::filesystem::remove(video_path_, ec);
    return;
  }

  stop_thread_ = true;
  // 退出时给队列中额外推入一个空帧，避免 pop 一直等待
  queue_.push({cv::Mat::zeros(0, 0, 0), {0, 0, 0, 0}, std::chrono::steady_clock::now()});
  if (saving_thread_.joinable()) saving_thread_.join();

  text_writer_.close();
  video_writer_.release();

  // ⭐⭐ W48 ②：**退出时报告**（原来是静默的）
  const int64_t w = written_.load(), d = dropped_.load();
  if (d > 0) {
    tools::logger()->warn(
      "[Recorder] 结束：写入 {} 帧，⚠️ **丢弃 {} 帧（{:.1f}%）** → {} 是**跳帧**的",
      w, d, 100.0 * d / (w + d > 0 ? w + d : 1), video_path_);
  } else {
    tools::logger()->info("[Recorder] 结束：写入 {} 帧，无丢帧 → {}", w, video_path_);
  }
}

void Recorder::save_to_file()
{
  // ⭐⭐ W82：录像编码最贵（~1 核）→ 用 SCHED_IDLE 避免抢自瞄
  tools::set_idle_policy();
  while (!stop_thread_) {
    FrameData frame;
    queue_.pop(frame);   // 从队列中取出帧数据
    if (frame.img.empty()) {
      if (stop_thread_) break;   // ⭐ 退出哨兵
      continue;
    }
    video_writer_.write(frame.img);
    ++written_;

    // 写入文本文件（输出顺序为 wxyz）
    Eigen::Vector4d xyzw = frame.q.coeffs();
    const auto since_begin = tools::delta_time(frame.timestamp, start_time_);
    text_writer_ << fmt::format(
      "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  }

  // ⭐ 把队列里剩下的排空（退出时别丢数据）
  FrameData frame;
  while (queue_.try_pop(frame)) {
    if (frame.img.empty()) continue;
    video_writer_.write(frame.img);
    ++written_;
    Eigen::Vector4d xyzw = frame.q.coeffs();
    const auto since_begin = tools::delta_time(frame.timestamp, start_time_);
    text_writer_ << fmt::format(
      "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  }
}

void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp)
{
  // ⭐⭐ W48 ①：**没开就直接返回**（原来无条件录）
  if (!enabled_) return;
  if (img.empty()) return;
  if (!init_) init(img);

  const auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;   // fps 节流

  last_time_ = timestamp;
  queue_.push({img, q, timestamp});      // 浅拷贝（cv::Mat 引用计数）；满时走 full_handler
}

void Recorder::init(const cv::Mat & img)
{
  text_writer_.open(text_path_);
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img.size());
  saving_thread_ = std::thread(&Recorder::save_to_file, this);
  init_ = true;
}

}  // namespace tools
