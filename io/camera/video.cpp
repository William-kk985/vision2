#include "io/camera/video.hpp"

#include <stdexcept>

#include "utils/log/logger.hpp"

namespace io
{

VideoCamera::VideoCamera(const std::string & video_path, double speed)
: speed_(speed <= 0 ? 1.0 : speed), t0_(std::chrono::steady_clock::now())
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
  // 合成时间戳：按视频帧率推进（再乘 speed 用于加速跑批）
  timestamp = t0_ + std::chrono::microseconds(
                      static_cast<int64_t>(idx_ * 1e6 / (fps_ * speed_)));
  ++idx_;
}

}  // namespace io
