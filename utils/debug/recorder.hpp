/**
 * @file utils/debug/recorder.hpp
 * @brief ⭐ 录像 + 位姿记录（学哈工程的 `bag_recorder`）
 *
 * 产物两个文件（同一时间戳命名）：
 * ```
 *   records/<时间戳>.avi   MJPG 视频
 *   records/<时间戳>.txt   每帧位姿 `since_begin w x y z`   ← ⭐ 标定 R 靠它
 * ```
 *
 * ## ⭐⭐⭐ W48 修了三个问题
 *
 * ### ① **无条件录 → 显式开启**
 * 原来 `Recorder` 构造函数就定好路径，主循环**无条件** `record()` →
 * ⭐ **跑任何程序都在录**（实测跑几十次测试就攒了 48 个文件）。
 * 现在：默认 `enabled = false`，要 `--record` 或 `set_enabled(true)` 才录。
 *
 * ### ② **静默丢帧 → 计数 + 警告 + 退出报告**
 * `queue_` 容量只有 1，`PopWhenFull=false`（丢新帧），
 * 且 `full_handler_` 是**空 lambda** → ⚠️ **worker 编码跟不上时静默丢帧**，
 * 录出来的视频**跳帧而你完全不知道**（MJPG 1080p 实测 26.5 ms/帧 ≈ 38 Hz）。
 * 现在：**首次丢帧 warn 一次 + 累计计数 + 退出时打印「录 N 帧 / 丢 M 帧」**。
 *
 * ### ③ **队列容量 1 → 可配（默认 8）**
 * 容量 1 意味着**工人线程只要慢一帧就丢**。默认放宽到 8（内存代价 8×4.4 MB ≈ 35 MB）。
 * ⚠️ 仍会丢（这是**有意的**：不学 `ImageSink` 那样无限堆积直到 OOM）。
 */
#ifndef HZMIR_UTILS_DEBUG_RECORDER_HPP
#define HZMIR_UTILS_DEBUG_RECORDER_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "utils/concurrency/thread_safe_queue.hpp"

namespace tools
{

class Recorder
{
public:
  /// @param fps 录制帧率（按它节流；默认 30 → worker 只需 33 ms/帧）
  /// @param enabled ⭐ **默认 `false`** —— 要录得显式开（`--record`）
  /// @param queue_cap ⭐ 队列容量（默认 8；原来是 1，慢一帧就丢）
  explicit Recorder(double fps = 30, bool enabled = false, size_t queue_cap = 8);
  ~Recorder();

  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp);

  /// @brief ⭐ 运行期开关（可绑热键）
  void set_enabled(bool on) { enabled_ = on; }
  bool enabled() const { return enabled_; }

  /// @brief ⭐ **幂等关闭**：flush 队列 + join worker + 报告帧数
  ///        测试/需要提前知道帧数时调用；析构会自动再调一次（幂等）
  void close();

  /// @brief 已写入的帧数 / 因队列满丢弃的帧数（诊断用）
  ///        ⚠️ 要在 `close()` **之后**才准确（worker 线程累加）
  int64_t written() const { return written_; }
  int64_t dropped() const { return dropped_; }

private:
  struct FrameData
  {
    cv::Mat img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  bool init_ = false;
  std::atomic<bool> stop_thread_{false};
  std::atomic<bool> enabled_{false};
  double fps_;
  std::string text_path_;
  std::string video_path_;
  std::ofstream text_writer_;
  cv::VideoWriter video_writer_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_time_;

  tools::ThreadSafeQueue<FrameData> queue_;
  std::thread saving_thread_;

  // ⭐⭐ W48：丢帧可观测（原来静默）
  std::atomic<int64_t> written_{0};
  std::atomic<int64_t> dropped_{0};
  std::atomic<bool> drop_warned_{false};

  std::atomic<bool> closed_{false};   // ⭐ close() 幂等
  void init(const cv::Mat & img);
  void save_to_file();
};

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_RECORDER_HPP
