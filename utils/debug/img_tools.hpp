#ifndef TOOLS__IMG_TOOLS_HPP
#define TOOLS__IMG_TOOLS_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace tools
{
void draw_point(
  cv::Mat & img, const cv::Point & point, const cv::Scalar & color = {0, 0, 255}, int radius = 3);

void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

/// @brief ⭐⭐ W113：**瞄准点标记**（外圈 + 实心内圆 + 十字准星）—— 照搬旧版赫兹
///   - W115 NOTE: currently NOT called anywhere. User asked to drop the
///     crosshair/circle (red box + a dot is enough), so the main loop now
///     draws the dot with cv::circle. Kept as a tool (old HEU version had it).
///   ⚠️ 用途：把"最终瞄哪"画在图上（旧版 `Hz_Rm_Vision/src/apps/infantry.cpp` 有这个，
///     新版重构时**没搬过来** ⇒ 只能看到检测框，看不到瞄准结果）。
///   ⭐ 纯几何绘制，**不依赖任何算法状态** ⇒ 放在 `img_tools` 里最合适。
void draw_aim_point(
  cv::Mat & img, const cv::Point & center, const cv::Scalar & color = {0, 0, 255}, int radius = 20,
  int thickness = 3);

/// @brief 重载：给四点，内部先算中心
void draw_aim_point(
  cv::Mat & img, const std::vector<cv::Point2f> & armor_points,
  const cv::Scalar & color = {0, 0, 255}, int radius = 20, int thickness = 3);

void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point,
  const cv::Scalar & color = {0, 255, 255}, double font_scale = 1.0, int thickness = 2);

}  // namespace tools

#endif  // TOOLS__IMG_TOOLS_HPP