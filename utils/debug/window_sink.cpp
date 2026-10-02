#include "utils/debug/window_sink.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdlib>

#include "utils/log/logger.hpp"

namespace tools
{

WindowSink::WindowSink(std::string window, double scale)
: window_(std::move(window)), scale_(scale > 0.05 ? scale : 1.0)
{
  if (!available()) {
    tools::logger()->warn(
      "[WindowSink] 当前环境无显示（DISPLAY 未设置或 OpenCV 无 GUI 后端）→ 按键 1 不会开窗");
    return;
  }
  cv::namedWindow(window_, cv::WINDOW_NORMAL);
  opened_ = true;
  tools::logger()->info("[WindowSink] 窗口 '{}' 已开（scale={:.2f}）", window_, scale_);
}

WindowSink::~WindowSink()
{
  if (!opened_) return;
  opened_ = false;
  // ⭐⭐ W76：真销毁 X 窗口。OpenCV 的 `destroyWindow` 只把销毁请求排队，
  //   必须再 pump 一次事件循环（`waitKey`）才会真正消失。
  try {
    cv::destroyWindow(window_);
    cv::waitKey(1);
  } catch (const std::exception & e) {
    tools::logger()->warn("[WindowSink] 销毁窗口 '{}' 失败: {}", window_, e.what());
  }
}

bool WindowSink::available()
{
  if (const char * d = std::getenv("DISPLAY"); !d || !*d) return false;
  const std::string info = cv::getBuildInformation();
  return info.find("GUI:") != std::string::npos &&
         info.find("GUI:                           NONE") == std::string::npos;
}

void WindowSink::on_frame(const auto_aim::FrameDebug & d)
{
  frame_id_ = d.frame_id;
  if (!opened_) return;
  // 让 waitKey 有机会处理窗口事件（否则窗口会「假死」）
  cv::waitKey(1);
}

void WindowSink::on_image(std::string_view tag, const cv::Mat & img, int64_t t_us)
{
  if (!opened_ || img.empty()) return;
  (void)t_us;

  cv::Mat shown;
  if (scale_ < 0.999)
    cv::resize(img, shown, {}, scale_, scale_, cv::INTER_AREA);
  else
    shown = img;

  cv::imshow(window_, shown);
  cv::setWindowTitle(window_, cv::format("%s | frame %u | %s", window_.c_str(), frame_id_,
                                         std::string(tag).c_str()));
  cv::waitKey(1);   // 必须：驱动 GUI 事件循环
}

}  // namespace tools
