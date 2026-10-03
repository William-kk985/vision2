#ifndef AUTO_AIM__DETECTOR_HPP
#define AUTO_AIM__DETECTOR_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "core/types.hpp"
#include "core/auto_aim/classifier/classifier.hpp"
#include "core/auto_aim/detector/detector_debug.hpp"

namespace auto_aim
{

/// ⭐⭐ W98：`detect()` 的返回值 —— **识别结果 + 调试快照一起返回**。
///
/// ## 为什么（8 个角色 debug 头里都写着的"下一步可做"）
/// 原来调试数据的流法是「角色算完 → 主循环手工从各处取值填 `FrameDebug`」：
/// ```
/// auto armors = yolo.detect(img);
/// fd.detector.armor_count = armors.size();          // ⚠️ 手写
/// const auto & st = auto_aim::last_detect_stats();  // ⚠️ 还要读一个全局旁路
/// fd.detector.nms_survivors = st.nms_survivors;
/// ```
/// ⇒ 实测后果：**`best_confidence` / `nms_survivors` 从未被赋值**（W64），
///   `fd.target.*` 13 列永远是 0（W70）—— 每修一次要改 **4 个主循环**。
///
/// 现在：**角色在返回时把 dbg 一起带出来** ⇒ 主循环一行 `fd.detector = r.dbg;`，
/// **结构上不可能忘填**（不填就编译不过）。
struct DetectorResult
{
  std::list<Armor> armors;
  DetectorDebug dbg;
};

class Detector
{
public:
  Detector(const std::string & config_path, bool debug = true);

  DetectorResult detect(const cv::Mat & bgr_img, int frame_count = -1);

  bool detect(Armor & armor, const cv::Mat & bgr_img);

  friend class YOLOV8;

private:
  Classifier classifier_;

  double threshold_;
  double max_angle_error_;
  double min_lightbar_ratio_, max_lightbar_ratio_;
  double min_lightbar_length_;
  double min_armor_ratio_, max_armor_ratio_;
  double max_side_ratio_;
  double min_confidence_;
  double max_rectangular_error_;

  bool debug_;
  std::string save_path_;

  // 利用PCA回归角点，参考自https://github.com/CSU-FYT-Vision/FYT2024_vision
  void lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const;

  bool check_geometry(const Lightbar & lightbar) const;
  bool check_geometry(const Armor & armor) const;
  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  Color get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const;
  cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor) const;
  ArmorType get_type(const Armor & armor);
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  void save(const Armor & armor) const;
  void show_result(
    const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
    const std::list<Armor> & armors, int frame_count) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__DETECTOR_HPP