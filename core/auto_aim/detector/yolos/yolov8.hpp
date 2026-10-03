#ifndef AUTO_AIM__YOLOV8_HPP
#define AUTO_AIM__YOLOV8_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "core/types.hpp"
#include "core/auto_aim/classifier/classifier.hpp"
#include "core/auto_aim/detector/detector.hpp"
#include "core/auto_aim/detector/yolo.hpp"

namespace auto_aim
{

class YOLOV8 : public YOLOBase
{
public:
  YOLOV8(const std::string & config_path, bool debug);

  DetectorResult detect(const cv::Mat & bgr_img, int frame_count) override;   // ⭐ W98

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  Classifier classifier_;
  Detector detector_;

  std::string device_, model_path_;
  std::string save_path_, debug_path_;
  bool debug_, use_roi_;

  const int class_num_ = 2;
  // ⭐⭐ W76：原来这两个是 **`const` 硬编码**（0.7 / 0.3），yaml 里改不了 ——
  //   调试时想放宽 `score_threshold` 看"差多少"完全做不到。
  //   ⇒ 现在可从 yaml 读，**默认值仍是同济原值 0.7 / 0.3**（不写这一项 = 同济行为）。
  //   ⚠️ 注意两关**串联**：`score_threshold_`（YOLO 解析）→ `min_confidence_`（check_name，默认 0.8）
  //   ⇒ **有效阈值是两者中更严的那个**（默认情形下是 0.8）。
  float nms_threshold_ = 0.3;
  float score_threshold_ = 0.7;
  double min_confidence_, binary_threshold_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;

  cv::Rect roi_;
  cv::Point2f offset_;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor) const;
  ArmorType get_type(const Armor & armor);
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;
  DetectorResult parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);   // ⭐ W98

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  void sort_keypoints(std::vector<cv::Point2f> & keypoints);
};

}  // namespace auto_aim

#endif  // TOOLS__YOLOV8_HPP