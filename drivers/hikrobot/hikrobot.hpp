#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "MvCameraControl.h"
#include "io/camera/camera.hpp"
#include "utils/concurrency/thread_safe_queue.hpp"

namespace io
{
class HikRobot : public CameraBase
{
public:
  /// @param fps ⭐⭐ W58：**目标帧率（可配）** —— 原来硬编码 150。
  ///   ⚠️ 150 fps @ 1440×1080 Bayer8 ≈ **233 MB/s**，接近 USB3 实际上限；
  ///   若协商到 USB2（480 Mbps ≈ 40 MB/s）则**一帧都传不过来** → `0x80000007`。
  ///   yaml 里加 `fps: 30` 可控；缺省 30（安全值）。
  HikRobot(double exposure_ms, double gain, const std::string & vid_pid, double fps = 30.0);
  ~HikRobot() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  double exposure_us_;
  double gain_;
  double fps_ = 30.0;   // ⭐ W58

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_;

  // ⭐⭐⭐ W56：**必须初始化** —— 同济原版是裸的 `void * handle_;`（垃圾值）。
  //   当 `capture_start()` 在 `MV_CC_CreateHandle` 之前就失败（如 `Not found camera!`）时，
  //   后续 `capture_stop()` 会拿**未初始化的 handle** 去调 `MV_CC_StopGrabbing` → **UB**，
  //   可能操作到不该操作的设备（实测：相机被留在 grabbing → **红灯常亮**）。
  void * handle_ = nullptr;
  std::thread capture_thread_;
  std::atomic<bool> capturing_;
  std::atomic<bool> capture_quit_;
  tools::ThreadSafeQueue<CameraData> queue_;

  int vid_, pid_;

  void capture_start();
  void capture_stop();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, unsigned int value);

  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP