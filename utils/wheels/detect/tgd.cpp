#include "utils/wheels/detect/tgd.hpp"

#include <numeric>
#include <cmath>

#include "utils/log/logger.hpp"

namespace tools
{

TGDDetector::TGDDetector(const TGDConfig& config)
: config_(config), frame_count_(0)
{
  tools::logger()->info("[TGD] Initialized with history_size={}, threshold={}", 
                        config_.history_size, config_.gradient_threshold);
}

void TGDDetector::reset()
{
  reference_frame_.release();
  prev_gradient_.release();
  history_.clear();
  frame_count_ = 0;
  
  tools::logger()->info("[TGD] Detector reset");
}

void TGDDetector::setConfig(const TGDConfig& config)
{
  config_ = config;
  tools::logger()->info("[TGD] Config updated: threshold={}, adaptive={}", 
                        config_.gradient_threshold, config_.use_adaptive_threshold);
}

cv::Mat TGDDetector::computeGradient(const cv::Mat& gray_img)
{
  // 使用 Sobel 算子计算梯度
  cv::Mat grad_x, grad_y;
  cv::Sobel(gray_img, grad_x, CV_16S, 1, 0, 3);
  cv::Sobel(gray_img, grad_y, CV_16S, 0, 1, 3);
  
  // 计算梯度幅值
  cv::Mat gradient = cv::abs(grad_x) + cv::abs(grad_y);
  gradient.convertTo(gradient, CV_8U);
  
  return gradient;
}

cv::Mat TGDDetector::computeTemporalGradientDifference(
  const cv::Mat& current_gradient,
  const cv::Mat& reference_gradient)
{
  if (current_gradient.empty() || reference_gradient.empty()) {
    return cv::Mat();
  }
  
  // 计算梯度差的绝对值
  cv::Mat diff;
  cv::absdiff(current_gradient, reference_gradient, diff);
  
  // 高斯模糊去除噪声
  cv::GaussianBlur(diff, diff, cv::Size(5, 5), 1.5);
  
  return diff;
}

cv::Mat TGDDetector::adaptiveThreshold(const cv::Mat& diff_img)
{
  cv::Mat binary;
  
  if (config_.use_adaptive_threshold) {
    // 计算图像的统计特性
    cv::Scalar mean_val, std_dev;
    cv::meanStdDev(diff_img, mean_val, std_dev);
    
    // 自适应阈值 = 均值 + k * 标准差
    double threshold = mean_val[0] + config_.threshold_scale * std_dev[0];
    threshold = std::max(threshold, config_.gradient_threshold);
    
    LOG_TGD("[TGD] Adaptive threshold: {:.2f} (mean={:.2f}, std={:.2f})", 
                           threshold, mean_val[0], std_dev[0]);
    
    // 固定阈值二值化
    cv::threshold(diff_img, binary, threshold, 255, cv::THRESH_BINARY);
  } else {
    // 使用固定阈值
    cv::threshold(diff_img, binary, config_.gradient_threshold, 255, cv::THRESH_BINARY);
  }
  
  return binary;
}

cv::Mat TGDDetector::morphologicalProcessing(const cv::Mat& binary_img)
{
  // 创建形态学核
  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_RECT, 
    cv::Size(config_.morph_kernel_size, config_.morph_kernel_size));
  
  // 闭运算：填充小孔洞
  cv::Mat closed;
  cv::morphologyEx(binary_img, closed, cv::MORPH_CLOSE, kernel, 
                   cv::Point(-1, -1), config_.morph_iterations);
  
  // 开运算：去除小噪点
  cv::Mat opened;
  cv::morphologyEx(closed, opened, cv::MORPH_OPEN, kernel, 
                   cv::Point(-1, -1), config_.morph_iterations);
  
  return opened;
}

std::vector<cv::Point2f> TGDDetector::extractCenters(const cv::Mat& binary_img)
{
  std::vector<cv::Point2f> centers;
  
  // 查找连通区域
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  for (const auto& contour : contours) {
    // 面积过滤
    double area = cv::contourArea(contour);
    if (area < config_.min_area) {
      continue;
    }
    
    // 计算矩心
    cv::Moments m = cv::moments(contour);
    if (m.m00 > 0) {
      cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
      centers.push_back(center);
    }
  }
  
  return centers;
}

void TGDDetector::updateReferenceFrame(const cv::Mat& gray_img)
{
  if (history_.empty()) {
    // 第一帧直接作为参考
    reference_frame_ = gray_img.clone();
    prev_gradient_ = computeGradient(reference_frame_);
  } else {
    // 使用滑动平均更新参考帧
    double alpha = 1.0 / (history_.size() + 1);
    cv::addWeighted(gray_img, alpha, reference_frame_, 1.0 - alpha, 0, reference_frame_);
  }
  
  // 添加到历史队列
  history_.push_back(gray_img.clone());
  if (static_cast<int>(history_.size()) > config_.history_size) {
    history_.pop_front();
  }
}

double TGDDetector::computeMotionIntensity(const cv::Mat& binary_img)
{
  // 计算运动区域的像素比例
  int total_pixels = binary_img.total();
  int motion_pixels = cv::countNonZero(binary_img);
  
  double intensity = static_cast<double>(motion_pixels) / total_pixels;
  return std::min(intensity * 10.0, 1.0);  // 归一化到 0-1
}

auto TGDDetector::process(const cv::Mat& bgr_img, double timestamp) -> TGDResult
{
  TGDResult result;
  result.frame_count = ++frame_count_;
  
  // 转换为灰度图
  cv::Mat gray_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
  
  // 初始化参考帧
  if (reference_frame_.empty()) {
    updateReferenceFrame(gray_img);
    tools::logger()->info("[TGD] Reference frame initialized (frame {})", frame_count_);
    return result;
  }
  
  // 1. 计算当前帧梯度
  cv::Mat current_gradient = computeGradient(gray_img);
  
  // 2. 计算时序梯度差
  cv::Mat diff_img = computeTemporalGradientDifference(
    current_gradient, prev_gradient_);
  
  // 3. 自适应阈值分割
  cv::Mat binary_img = adaptiveThreshold(diff_img);
  
  // 4. 形态学后处理
  cv::Mat enhanced_binary = morphologicalProcessing(binary_img);
  
  // 5. 提取中心点
  result.centers = extractCenters(enhanced_binary);
  
  // 6. 计算运动强度
  result.motion_intensity = computeMotionIntensity(enhanced_binary);
  result.has_motion = result.motion_intensity > 0.05;  // 5% 阈值
  
  // 7. 更新参考帧（仅在场景变化不大时）
  if (result.motion_intensity < 0.3) {
    updateReferenceFrame(gray_img);
    prev_gradient_ = current_gradient;
  }
  
  // 保存结果
  result.enhanced_binary = enhanced_binary.clone();
  
  // 调试日志
  if (config_.debug_mode) {
    LOG_TGD(
      "[TGD] Frame {}: centers={}, motion_intensity={:.2f}%",
      frame_count_, result.centers.size(), result.motion_intensity * 100);
  }
  
  return result;
}

}  // namespace tools
