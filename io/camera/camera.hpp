#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

class Camera : public CameraBase
{
public:
  /// @param dump_params ⭐ W90：只打印相机常用参数的当前值（用于把 MVS 里的好值
  ///   抄进 yaml 的 `camera_params`）—— 不用每次开 MVS
  explicit Camera(const std::string & config_path, bool dump_params = false);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP