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
  /// @param dump_params   ⭐ W90：只打印相机常用参数的当前值（用于把 MVS 里的好值
  ///   抄进 yaml）—— 不用每次开 MVS
  /// @param camera_config ⭐⭐ W92：**相机专配 yaml**（4 兵种共用，默认 `params/camera.yaml`）。
  ///   优先级：**兵种 yaml 的相机段 > camera.yaml > 内置默认**
  ///   ⇒ 常用参数只改 camera.yaml 一处；某个兵种要特殊值，就在自己的 yaml 里覆盖。
  ///   传空字符串 `""` = 不用专配（只读兵种 yaml，兼容原行为）。
  explicit Camera(
    const std::string & config_path, bool dump_params = false,
    const std::string & camera_config = "params/camera.yaml");
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP