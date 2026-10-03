/**
 * @file core/auto_aim/detector/detector_slot.hpp
 * @brief ⭐⭐⭐ **检测器统一槽位**：`detector_impl: yolo | traditional`（W101 · 原 B2）
 *
 * ## 改前的问题
 * 本项目有**两个完全独立的检测器**，但**选哪个是硬编码在兵种 `.cpp` 里的**：
 * ```
 * infantry / hero / sentry :  core/auto_aim/detector/yolo.hpp    ← YOLO（yolov5/v8/11）
 * uav                      :  core/auto_aim/detector/detector.hpp ← 传统（灯条配对 + 分类器）
 * ```
 * ⚠️ 后果：
 *   · 想在步兵上试试传统检测器（对比命中率）⇒ **要改代码重编**
 *   · `yolo_name` 这个 yaml 键**在 4 个兵种的 yaml 里都写了**，
 *     但对 uav **完全无效**（它根本不读）⇒ **误导**
 *
 * ## 现在
 * yaml 里写一行：
 * ```yaml
 * detector_impl: yolo          # 默认（= 同济：步兵/英雄/哨兵用 YOLO）
 * # detector_impl: traditional # 传统灯条配对
 * ```
 * ⭐ **留空 = `yolo`**，所以现有 yaml **一行都不用改**行为就不变（同济为准）。
 *
 * ## 为什么能做得这么薄
 * 两个类的接口**恰好一致**（都是 `(const std::string& config_path, bool debug)` +
 * `DetectorResult detect(const cv::Mat&, int)`），所以不需要"适配器"，
 * 一个 `std::variant` 风格的槽位就够。
 *
 * ## ⚠️ 只在构造时**建选中的那个**
 * 两个检测器都要加载模型/分类器，**同时建两个是纯浪费** ⇒ 惰性建。
 */
#ifndef HZMIR_CORE_AUTO_AIM_DETECTOR_DETECTOR_SLOT_HPP
#define HZMIR_CORE_AUTO_AIM_DETECTOR_DETECTOR_SLOT_HPP

#include <memory>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "core/auto_aim/detector/detector.hpp"   // 传统检测器
#include "core/auto_aim/detector/yolo.hpp"       // YOLO
#include "utils/log/logger.hpp"

namespace auto_aim
{

/// @brief 检测器槽位：按 `detector_impl` 建对应的实现，对外只有 `detect()`
class DetectorSlot
{
public:
  /// @param config_path 兵种 yaml 路径（两个检测器都用它初始化）
  /// @param debug       是否画检测框（L3 用）
  explicit DetectorSlot(const std::string & config_path, bool debug = true)
  {
    YAML::Node y;                       // ⭐ 读不到 yaml 也不该崩：按默认 yolo 走
    try {
      y = YAML::LoadFile(config_path);
    } catch (const std::exception & e) {
      tools::logger()->warn("[DetectorSlot] 读不到 {}: {} → 按默认 yolo", config_path, e.what());
    }
    impl_ = (y && y["detector_impl"]) ? y["detector_impl"].as<std::string>() : "yolo";

    if (impl_ == "yolo") {
      yolo_ = std::make_unique<YOLO>(config_path, debug);
    } else if (impl_ == "traditional" || impl_ == "trad" || impl_ == "tgd") {
      impl_ = "traditional";            // ⭐ 归一化别名，日志里只出现规范名
      traditional_ = std::make_unique<Detector>(config_path, debug);
    } else {
      // ★ 不静默回退（与 trajectory_impl 一致的做法）
      throw std::runtime_error(
        "未知的 detector_impl: \"" + impl_ +
        "\"（可选 yolo / traditional；留空= yolo）");
    }
    tools::logger()->info("[DetectorSlot] detector_impl = {}", impl_);
  }

  /// @brief 统一入口（两个实现的签名本来就一致）
  DetectorResult detect(const cv::Mat & img, int frame_count = -1)
  {
    if (yolo_) return yolo_->detect(img, frame_count);
    return traditional_->detect(img, frame_count);
  }

  /// @brief 实际生效的实现名（规范名）
  const std::string & impl() const { return impl_; }

private:
  std::string impl_;
  std::unique_ptr<YOLO> yolo_;
  std::unique_ptr<Detector> traditional_;
};

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_DETECTOR_DETECTOR_SLOT_HPP
