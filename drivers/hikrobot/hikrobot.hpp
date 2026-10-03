#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

#include "MvCameraControl.h"
#include "io/camera/camera.hpp"
#include "utils/concurrency/thread_safe_queue.hpp"

namespace io
{

/// ⭐⭐ W90：**通用相机参数** —— 让用户不用开 MVS 就能调任意海康参数。
/// yaml 里写：
/// ```yaml
/// camera_params:
///   float:  { Gamma: 1.0, Sharpness: 50 }     # MV_CC_SetFloatValue
///   enum:   { BalanceWhiteAuto: 0 }           # MV_CC_SetEnumValue
///   int:    { Width: 1280 }                   # MV_CC_SetIntValue
/// ```
/// ⚠️ 参数名必须与 **MVS 客户端里显示的名字一致**（如 `ExposureTime` / `Gain` /
///   `BalanceWhiteAuto` / `Gamma` / `Sharpness` / `AcquisitionFrameRate` …）。
/// ⭐ 用 `--dump-camera-params` 可以把常用参数**当前值全部打印出来**，
///   先在 MVS 里调好、再把好用的值抄进 yaml 即可。
struct CameraParam
{
  enum Kind { Float, Enum, Int } kind = Float;
  std::string name;
  double fval = 0;    ///< kind==Float
  int64_t ival = 0;   ///< kind==Enum / Int
};

class HikRobot : public CameraBase
{
public:
  /// @param fps ⭐⭐ W58：**目标帧率（可配）** —— 原来硬编码 150。
  ///   ⚠️ 150 fps @ 1440×1080 Bayer8 ≈ **233 MB/s**，接近 USB3 实际上限；
  ///   若协商到 USB2（480 Mbps ≈ 40 MB/s）则**一帧都传不过来** → `0x80000007`。
  ///   yaml 里加 `fps: 30` 可控；缺省 30（安全值）。
  /// @param extra        ⭐ W90：额外参数（来自 yaml 的 `camera_params`），**最后应用**
  /// @param dump_params  ⭐ 只打印常用参数的当前值然后退出（不要在 MVS 里瞎猜名字）
  HikRobot(
    double exposure_ms, double gain, const std::string & vid_pid, double fps = 30.0,
    const std::vector<CameraParam> & extra = {}, bool dump_params = false);
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
  void set_int_value(const std::string & name, int64_t value);          // ⭐ W90
  void apply_extra_params(const std::vector<CameraParam> & extra);      // ⭐ W90
  void dump_camera_params() const;                                      // ⭐ W90

  std::vector<CameraParam> extra_;   // ⭐ W90
  bool dump_params_ = false;         // ⭐ W90

  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP