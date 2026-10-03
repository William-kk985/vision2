/**
 * @file io/camera/video.hpp
 * @brief ⭐ 录像回放相机（doc 09 §4「io/camera/virtual/」）
 *
 * **零硬件验证的前提**：没有相机时，用一段 `.avi` 驱动完整主链路。
 *
 * 配套位姿文件（可选）：与视频同名的 `.txt`，每行 `<t> <w> <x> <y> <z>`
 *   （秒级相对时间 + 云台四元数 wxyz）—— 见 `io::ReplayBoard`。
 */
#ifndef HZMIR_IO_CAMERA_VIDEO_HPP
#define HZMIR_IO_CAMERA_VIDEO_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera/camera.hpp"

namespace io
{

class VideoCamera : public CameraBase
{
public:
  /// @param video_path 视频文件路径（.avi/.mp4/...）
  /// @param speed      播放速率：`1.0` = 实时（30fps 录像就 30fps）；`4.0` = 4 倍速；
///                   ⭐ `0` = **不节流，全速跑批**（测吞吐/批量回归用）
  explicit VideoCamera(const std::string & video_path, double speed = 1.0);

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

  bool ok() const { return cap_.isOpened(); }
  double fps() const { return fps_; }
  int frame_count() const { return total_; }

private:
  cv::VideoCapture cap_;
  double speed_ = 1.0;
  double fps_ = 30.0;
  int total_ = 0;
  int idx_ = 0;
  std::chrono::steady_clock::time_point t0_;
};

}  // namespace io

#endif  // HZMIR_IO_CAMERA_VIDEO_HPP
