/**
 * @file utils/debug/window_sink.hpp
 * @brief ⭐ L3：可视化窗口 sink（doc 09 §14「昂贵通道」，按键 `1`）
 *
 * ⚠️ **L3 是唯一允许用宏的层** —— 本文件本身不带宏，宏只出现在**装配处**（主程序）。
 *
 * ⚠️ OpenCV 的 GUI 不是线程安全的 → 本 sink **必须**在主循环线程被调用（同步 `imshow`）。
 * 无显示环境（无 `DISPLAY`）时 `available()==false`，装配方应跳过并提示，**不静默失败**。
 */
#ifndef HZMIR_UTILS_DEBUG_WINDOW_SINK_HPP
#define HZMIR_UTILS_DEBUG_WINDOW_SINK_HPP

#include <string>

#include "debug_sink.hpp"

namespace tools
{

class WindowSink : public IDebugSink
{
public:
  /// @param window 窗口名
  /// @param scale  显示缩放（>0，小图省 CPU）
  explicit WindowSink(std::string window = "hzmir_debug", double scale = 0.5);

  const char * name() const override { return "window"; }
  bool wants_image() const override { return true; }

  void on_frame(const auto_aim::FrameDebug & d) override;   // 只缓存帧号，用于标题
  void on_image(std::string_view tag, const cv::Mat & img, int64_t t_us) override;

  /// @brief 当前环境能否开窗（有 DISPLAY 且 OpenCV 编译了 GUI 后端）
  static bool available();

  const std::string & window() const { return window_; }

private:
  std::string window_;
  double scale_ = 0.5;
  bool opened_ = false;
  uint32_t frame_id_ = 0;
};

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_WINDOW_SINK_HPP
