#include "img_tools.hpp"

namespace tools
{
void draw_point(cv::Mat & img, const cv::Point & point, const cv::Scalar & color, int radius)
{
  cv::circle(img, point, radius, color, -1);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)
{
  std::vector<std::vector<cv::Point>> contours = {points};
  cv::drawContours(img, contours, -1, color, thickness);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)
{
  std::vector<cv::Point> int_points(points.begin(), points.end());
  draw_points(img, int_points, color, thickness);
}

void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point, const cv::Scalar & color,
  double font_scale, int thickness)
{
  cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
}


// ⭐⭐ W113：瞄准点标记（外圈 + 内圆 + 十字）—— 照搬旧版赫兹
void draw_aim_point(
  cv::Mat & img, const cv::Point & center, const cv::Scalar & color, int radius, int thickness)
{
  cv::circle(img, center, radius, color, thickness);       // 外圈
  cv::circle(img, center, radius / 3, color, -1);          // 实心内圆
  const int cross_len = radius + 10;                       // 十字准星
  cv::line(img, {center.x - cross_len, center.y}, {center.x + cross_len, center.y}, color, thickness);
  cv::line(img, {center.x, center.y - cross_len}, {center.x, center.y + cross_len}, color, thickness);
}

void draw_aim_point(
  cv::Mat & img, const std::vector<cv::Point2f> & armor_points, const cv::Scalar & color,
  int radius, int thickness)
{
  if (armor_points.empty()) return;
  cv::Point2f c(0, 0);
  for (const auto & pt : armor_points) c += pt;
  c.x /= static_cast<float>(armor_points.size());
  c.y /= static_cast<float>(armor_points.size());
  draw_aim_point(img, cv::Point(static_cast<int>(c.x), static_cast<int>(c.y)), color, radius, thickness);
}
}  // namespace tools
