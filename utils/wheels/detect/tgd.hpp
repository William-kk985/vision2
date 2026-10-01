/**
 * @file utils/wheels/detect/tgd.hpp
 * @brief ⭐ 轮子：TGD（时序梯度差分）运动目标检测 —— 大连理工
 *
 * **来源**：`Hz_Rm_Vision/src/core/detection/tgd_detector.{hpp,cpp}`（155 + 223 行）
 * **落位理由**：`grep` 证明它**零业务类型依赖**（只有 OpenCV/STL）→ 属于 `utils/wheels/`
 * **命名空间**：`auto_aim` → `tools`（它本来就不该在业务命名空间里）
 * **内容改动**：仅 `logger` 路径 + 命名空间；**算法一行未动**（保留出处）
 *
 * 算法：梯度图 → 时序梯度差分 → 自适应阈值 → 形态学 → 连通域中心
 * 用途：对「帧间运动」敏感，可作为传统方法检测的补充通道
 */
#ifndef HZMIR_UTILS_WHEELS_DETECT_TGD_HPP
#define HZMIR_UTILS_WHEELS_DETECT_TGD_HPP

/**
 * @file tgd_detector.hpp
 * @brief 大连理工大学 TGD (Temporal Gradient Difference) 时序梯度差检测方法
 * 
 * 原理：利用连续帧之间的梯度差异来增强运动目标的检测
 * - 计算当前帧与参考帧的梯度差
 * - 通过阈值分割提取运动区域
 * - 结合形态学操作去除噪声
 * - 输出增强的二值图像用于后续装甲板检测
 */

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <memory>

namespace tools
{

/**
 * @brief TGD 配置参数
 */
struct TGDConfig
{
  // 基础参数
  int history_size = 3;              // 历史帧数量
  double gradient_threshold = 30.0;  // 梯度阈值
  double min_area = 50.0;            // 最小有效面积
  
  // 形态学参数
  int morph_kernel_size = 3;         // 形态学核大小
  int morph_iterations = 2;          // 形态学迭代次数
  
  // 自适应参数
  bool use_adaptive_threshold = true;  // 是否使用自适应阈值
  double threshold_scale = 1.5;        // 阈值缩放系数
  
  // 调试参数
  bool debug_mode = false;           // 调试模式
};

/**
 * @brief TGD 检测结果
 */
struct TGDResult
{
  cv::Mat enhanced_binary;           // 增强后的二值图像
  std::vector<cv::Point2f> centers;  // 检测到的中心点
  double motion_intensity = 0.0;     // 运动强度（0-1）
  bool has_motion = false;           // 是否检测到运动
  int frame_count = 0;               // 帧计数
};

/**
 * @brief TGD 检测器类
 */
class TGDDetector
{
public:
  /**
   * @brief 构造函数
   * @param config TGD 配置参数
   */
  explicit TGDDetector(const TGDConfig& config = TGDConfig());
  
  /**
   * @brief 处理单帧图像
   * @param bgr_img 输入 BGR 图像
   * @param timestamp 时间戳（可选，用于计算帧间隔）
   * @return TGD 检测结果
   */
  TGDResult process(const cv::Mat& bgr_img, double timestamp = -1.0);
  
  /**
   * @brief 获取参考帧（用于调试或外部使用）
   * @return 当前参考帧
   */
  cv::Mat getReferenceFrame() const { return reference_frame_; }
  
  /**
   * @brief 重置检测器状态
   */
  void reset();
  
  /**
   * @brief 更新配置参数
   * @param config 新的配置参数
   */
  void setConfig(const TGDConfig& config);

private:
  TGDConfig config_;
  cv::Mat reference_frame_;          // 参考帧（灰度）
  cv::Mat prev_gradient_;            // 上一帧梯度
  std::deque<cv::Mat> history_;      // 历史帧队列
  int frame_count_ = 0;              // 总帧数
  
  /**
   * @brief 计算图像的梯度幅值
   * @param gray_img 灰度图像
   * @return 梯度幅值图
   */
  cv::Mat computeGradient(const cv::Mat& gray_img);
  
  /**
   * @brief 计算时序梯度差
   * @param current_gradient 当前帧梯度
   * @param reference_gradient 参考帧梯度
   * @return 梯度差图像
   */
  cv::Mat computeTemporalGradientDifference(
    const cv::Mat& current_gradient,
    const cv::Mat& reference_gradient);
  
  /**
   * @brief 自适应阈值分割
   * @param diff_img 梯度差图像
   * @return 二值图像
   */
  cv::Mat adaptiveThreshold(const cv::Mat& diff_img);
  
  /**
   * @brief 形态学后处理
   * @param binary_img 二值图像
   * @return 处理后的图像
   */
  cv::Mat morphologicalProcessing(const cv::Mat& binary_img);
  
  /**
   * @brief 提取连通区域中心
   * @param binary_img 二值图像
   * @return 中心点列表
   */
  std::vector<cv::Point2f> extractCenters(const cv::Mat& binary_img);
  
  /**
   * @brief 更新参考帧
   * @param gray_img 当前灰度帧
   */
  void updateReferenceFrame(const cv::Mat& gray_img);
  
  /**
   * @brief 计算运动强度
   * @param binary_img 二值图像
   * @return 运动强度 (0-1)
   */
  double computeMotionIntensity(const cv::Mat& binary_img);
};

}  // namespace tools

#endif  // HZMIR_UTILS_WHEELS_DETECT_TGD_HPP
