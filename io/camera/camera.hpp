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
  /// @param camera_config ⭐⭐ W94：**覆盖相机配置路径**。
  ///   默认空 `""` ⇒ 用【兵种 yaml 的 `camera_config:` 键】指定的文件
  ///   （如 `params/robots/infantry.yaml` → `params/cameras/infantry.yaml`）。
  ///   ⭐ 由该文件的 `camera_name` 自动带出品牌文件 `params/cameras/<品牌>.yaml`。
  ///   CLI 显式给值时以 CLI 为准。
  explicit Camera(
    const std::string & config_path, bool dump_params = false,
    const std::string & camera_config = "");
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP