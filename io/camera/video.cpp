#include "io/camera/video.hpp"

#include <algorithm>
#include <thread>

#include <stdexcept>

#include "utils/log/logger.hpp"

namespace io
{

VideoCamera::VideoCamera(const std::string & video_path, double speed)
: speed_(speed < 0 ? 1.0 : speed)   // ⭐ W89：**0 = 不节流（全速跑批）**
  , t0_(std::chrono::steady_clock::now())
{
  cap_.open(video_path);
  if (!cap_.isOpened()) throw std::runtime_error("[VideoCamera] 打不开视频: " + video_path);

  fps_ = cap_.get(cv::CAP_PROP_FPS);
  if (fps_ <= 1.0) fps_ = 30.0;   // 有些封装读不到 fps
  total_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_COUNT));

  tools::logger()->info(
    "[VideoCamera] {} fps={:.1f} frames={} speed={:.1f}x", video_path, fps_, total_, speed_);
}

void VideoCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  cap_ >> img;

  // ⭐⭐⭐ W89 修复：**原来这里完全没有节流** —— `speed_` 只改变【合成时间戳】，
  //   不影响实际播放速率 ⇒ 循环跑到多快由处理速度决定（实测 ~98 fps），
  //   ⚠️ 症状：`--video-speed=1` 的 687 帧录像 6.98 s 就跑完了（应 22.9 s），
  //      看起来"被加速了"；而且**时间戳与墙钟严重脱节**（时间戳走到 22.9 s，墙钟才 7 s）。
  //
  //   ⇒ 现在按目标速率**节流**：睡到该帧时间戳对应的墙钟时刻。
  //      ⚠️ 处理慢于目标时不睡（`target > now` 才睡）—— 不会额外拖慢。
  //
  //   `speed_` 语义（与 `video.hpp` 文档一致）：
  //     · `> 0`：按 `fps × speed` 播放（1.0 = 实时；4.0 = 4 倍速快放）
  //     · `= 0`：⭐ **不节流，全速跑批**（测吞吐 / 批量回归用；时间戳仍按 1× 合成）
  timestamp = t0_ + std::chrono::microseconds(
                      static_cast<int64_t>(idx_ * 1e6 / (fps_ * std::max(1.0, speed_))));
  ++idx_;

  if (speed_ > 0.0) {
    const auto target = t0_ + std::chrono::microseconds(
                                  static_cast<int64_t>(idx_ * 1e6 / (fps_ * speed_)));
    const auto now = std::chrono::steady_clock::now();
    if (target > now) std::this_thread::sleep_until(target);
  }
}

}  // namespace io
