#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

#include "core/types.hpp"
#include "core/auto_aim/detector/detector.hpp"   // ⭐ W98：DetectorResult

namespace auto_aim
{
class YOLOBase
{
public:
  /// ⭐⭐⭐ W101 修复（**继承自同济的真缺陷**）：**必须有虚析构**。
  ///
  /// ⚠️ 原来 `YOLOBase` 没有任何析构函数，而 `YOLO` 用
  /// `std::unique_ptr<YOLOBase> yolo_` 持有子类（YOLOV5/V8/11）。
  /// C++ 标准：**通过基类指针 delete 非虚析构的派生类对象 = 未定义行为（UB）**
  /// —— 派生类的析构函数**不会被调用**，成员（含 `cv::dnn::Net`、OpenVINO 模型等）
  /// 的析构被跳过 ⇒ 资源泄漏；某些实现下还可能直接崩。
  /// ⇒ 加 `= default` 即可（子类无需额外动作，但会正确参与虚表）。
  virtual ~YOLOBase() = default;

  /// ⭐⭐ W98：返回 `DetectorResult`（结果 + 调试快照），见 `detector.hpp`
  virtual DetectorResult detect(const cv::Mat & img, int frame_count) = 0;

  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);

  DetectorResult detect(const cv::Mat & img, int frame_count = -1);

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP