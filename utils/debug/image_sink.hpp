/**
 * @file utils/debug/image_sink.hpp
 * @brief ⭐ L3：存图 sink（doc 09 §14「昂贵通道」，按键 `4`）
 *
 * ⚠️ **热路径不做 IO**（doc 09 §14.3 约束 1）：`on_image()` 只 `clone()` 后入队，
 *    `imwrite` 在**独立线程**里做。
 *
 * 节流：每 `every_n` 张存一张；超过 `max_files` 停止（**防止写爆磁盘**，同 W16 的教训）。
 */
#ifndef HZMIR_UTILS_DEBUG_IMAGE_SINK_HPP
#define HZMIR_UTILS_DEBUG_IMAGE_SINK_HPP

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "debug_sink.hpp"

namespace tools
{

class ImageSink : public IDebugSink
{
public:
  /// @param out_dir  输出目录（不存在会创建）
  /// @param every_n  每 N 张存一张（1 = 全存）
  /// @param max_files 上限（⭐ 安全网，超过就停并只警告一次）
  explicit ImageSink(std::string out_dir = "debug_imgs", int every_n = 30, size_t max_files = 500);
  ~ImageSink() override;

  const char * name() const override { return "image"; }
  bool wants_image() const override { return true; }

  void on_frame(const auto_aim::FrameDebug & d) override { frame_id_ = d.frame_id; }
  void on_image(std::string_view tag, const cv::Mat & img, int64_t t_us) override;

  // ⭐ W19：跨线程读 → 必须原子（原来是普通 size_t，data race）
  size_t saved() const { return saved_.load(); }
  size_t dropped() const { return dropped_.load(); }
  void close();

private:
  void worker();

  std::string out_dir_;
  int every_n_ = 30;
  size_t max_files_ = 500;
  uint32_t frame_id_ = 0;
  size_t seen_ = 0;
  // ⭐ W19：上限判断必须基于**只增不减**的已接受数。
  //    原来用 `saved_ + q_.size()`，而 `q_` 正被 worker 线程消费 → **data race** 且结果不确定。
  std::atomic<size_t> accepted_{0};
  std::atomic<size_t> saved_{0}, dropped_{0};
  bool capped_warned_ = false;

  std::thread th_;
  std::mutex mtx_;
  std::condition_variable cv_;
  bool quit_ = false;
  std::deque<std::pair<std::string, cv::Mat>> q_;
};

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_IMAGE_SINK_HPP
