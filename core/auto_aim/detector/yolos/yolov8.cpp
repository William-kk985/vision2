#include "yolov8.hpp"
#include "../det_stats.hpp"   // ⭐ W63：检测统计开关
#include "utils/debug/l3_gate.hpp"   // ⭐ W61：全局 L3 门控

#include <fmt/chrono.h>
#include <omp.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <random>

#include "utils/ov/device.hpp"   // ⭐ W28：device 回退
#include "core/auto_aim/classifier/classifier.hpp"
#include "utils/debug/img_tools.hpp"
#include "utils/log/logger.hpp"

namespace auto_aim
{
YOLOV8::YOLOV8(const std::string & config_path, bool debug)
: classifier_(config_path), detector_(config_path), debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolov8_model_path"].as<std::string>();
  // ⭐ W28：解析设备（不可用时回退 CPU，而不是抛异常）
  //   同济原句：`device_ = yaml["device"].as<std::string>();`
  device_ = tools::resolve_device(
    core_, yaml["device"] ? yaml["device"].as<std::string>() : "");
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);

  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 416, 416, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  // TODO: ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

std::list<Armor> YOLOV8::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width == -1) {  // -1 表示该维度不裁切
      roi_.width = raw_img.cols;
    }
    if (roi_.height == -1) {  // -1 表示该维度不裁切
      roi_.height = raw_img.rows;
    }
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  auto x_scale = static_cast<double>(416) / bgr_img.rows;
  auto y_scale = static_cast<double>(416) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // preproces
  auto input = cv::Mat(416, 416, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  ov::Tensor input_tensor(ov::element::u8, {1, 416, 416, 3}, input.data);

  /// infer
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLOV8::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // for each row: xywh + classess
  cv::transpose(output, output);

  std::vector<int> ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  // ⭐⭐⭐ W62：**逐帧统计各步过滤** —— 解决「识别不出来时不知道卡在哪」
  //   原来 objectness/not_armor/置信度/类型 四种失败【都没有日志】、
  //   `armor_count` 又是过滤后的 → 终端和 CSV 都分不出卡在哪一步。
  int n_pass = 0, n_name = 0, n_conf = 0, n_type = 0;   // ⭐ 只统计 objectness 通过后的候选
  // ⭐⭐ W66：记录本帧**最高 objectness**（以及它所在的 anchor 行），
  //   这样没候选时能打印「最高多少 / 阈值多少 / 它以为是哪个类别」——真实数值而非一句"没过"
  double max_obj = 0;
  int max_obj_row = -1;
  for (int r = 0; r < output.rows; r++) {
    auto xywh = output.row(r).colRange(0, 4);
    auto scores = output.row(r).colRange(4, 4 + class_num_);
    auto one_key_points = output.row(r).colRange(4 + class_num_, 14);

    std::vector<cv::Point2f> armor_key_points;

    double score;
    cv::Point max_point;
    cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);

      if (score > max_obj) { max_obj = score; max_obj_row = r; }   // ⭐ W66：便宜（一次比较）
      if (score < score_threshold_) continue;   // ⭐ 大部分 anchor 都在这（不是装甲板）
      ++n_pass;                                  // ⭐ 通过 objectness = "像装甲板"的候选

    auto x = xywh.at<float>(0);
    auto y = xywh.at<float>(1);
    auto w = xywh.at<float>(2);
    auto h = xywh.at<float>(3);
    auto left = static_cast<int>((x - 0.5 * w) / scale);
    auto top = static_cast<int>((y - 0.5 * h) / scale);
    auto width = static_cast<int>(w / scale);
    auto height = static_cast<int>(h / scale);

    for (int i = 0; i < 4; i++) {
      float x = one_key_points.at<float>(0, i * 2 + 0) / scale;
      float y = one_key_points.at<float>(0, i * 2 + 1) / scale;
      cv::Point2f kp = {x, y};
      armor_key_points.push_back(kp);
    }
    ids.emplace_back(max_point.x);
    confidences.emplace_back(score);
    boxes.emplace_back(left, top, width, height);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);
  const int nms_survivors = static_cast<int>(indices.size());   // ⭐ W64：NMS 存活数（原来算完就扔）

  std::list<Armor> armors;
  for (const auto & i : indices) {
    sort_keypoints(armors_key_points[i]);
    if (use_roi_) {
      armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  for (auto it = armors.begin(); it != armors.end();) {
    it->pattern = get_pattern(bgr_img, *it);
    classifier_.classify(*it);

    if (it->name == ArmorName::not_armor) { ++n_name; it = armors.erase(it); continue; }
    if (it->confidence <= min_confidence_) { ++n_conf; it = armors.erase(it); continue; }
    if (!check_name(*it)) {
      ++n_conf;
    }

    it->type = get_type(*it);
    if (!check_type(*it)) {
      ++n_type;
      it = armors.erase(it);
      continue;
    }

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  // ⭐⭐⭐ W62：**一帧一行汇总**（`n_raw == 0` 时说明 YOLO 根本没输出候选 → 不打）
  // ⭐⭐⭐ W62：**一帧一行汇总** —— 一眼看出卡在哪一步
  //   ⚠️ `n_pass` 才是"像装甲板的候选数"；anchor 总数（25200）没意义、不打。
  // ⭐⭐ W66：没候选时，把「最像的那个 anchor」的类别也解出来（**只算一次**，不是 25200 次）
  int best_cls = -1, best_color = -1;
  double best_cls_score = 0, best_color_score = 0;
  if (n_pass == 0 && max_obj_row >= 0) {
    cv::Mat cs = output.row(max_obj_row).colRange(13, 22);
    cv::Mat cols = output.row(max_obj_row).colRange(9, 13);
    cv::Point cid, coid;
    cv::minMaxLoc(cs, NULL, &best_cls_score, NULL, &cid);
    cv::minMaxLoc(cols, NULL, &best_color_score, NULL, &coid);
    best_cls = cid.x;
    best_color = coid.x;                    // ⭐ 颜色用它自己的 argmax 索引（原来错用了 best_cls）
    // ⚠️ 类别/颜色的输出是 **logit**（没 sigmoid）→ 显示前转成概率，否则数字会误导
    const auto sig = [](double x) { return 1.0 / (1.0 + std::exp(-x)); };   // ⭐ 本地，不依赖成员
    best_cls_score = sig(best_cls_score);
    best_color_score = sig(best_color_score);
  }

  // ⭐ 情况 0：**objectness 一个候选都没过** → YOLO 压根没看到"像装甲板的东西"
  if (auto_aim::det_stats_enabled() && n_pass == 0) {
    static int no_cand_count = 0;
    if (++no_cand_count % 60 == 1)   // 节流：每 60 帧（约 0.6 秒）报一次
      tools::logger()->debug(   // ⭐ W63：warn→debug
        "[YOLOV8] objectness 一个候选都没过（**本帧最高 {:.3f}** < 阈值 {:.2f}）"
        " ⇒ 没检出。最像的 anchor: 类别={}({:.1%}) 颜色={}({:.1%})；anchor {} 个",
        max_obj, score_threshold_,
        (best_cls >= 0 && best_cls < (int)ARMOR_NAMES.size()) ? ARMOR_NAMES[best_cls] : "?",
        best_cls_score,
        (best_color >= 0 && best_color < (int)COLORS.size()) ? COLORS[best_color] : "?",
        best_color_score, output.rows);
  }
  if (auto_aim::det_stats_enabled() && n_pass > 0) {
    static int last_n_out = -1;
    static int same_count = 0;
    const int n_out = static_cast<int>(armors.size());
    if (n_out == last_n_out && ++same_count % 30 != 0) {
      // 节流：输出数没变化时每 30 帧报一次（约 0.3 秒）
    } else {
      same_count = 0;
      last_n_out = n_out;
      if (n_out == 0)
        tools::logger()->debug(   // ⭐ W63：warn→debug（它是诊断信息不是告警），info 级别可静音
          "[{}] objectness 通过 {} 个候选 → **全被滤掉**：not_armor {} / 置信度 {} / "
          "类型不符 {}  ⇒ 最终 0 个装甲板",
          "yolov8", n_pass, n_name, n_conf, n_type);
      else
        tools::logger()->debug(
          "[{}] objectness 通过 {} → 输出 {}（滤掉 not_armor {} / conf {} / type {}）", "yolov8",
          n_pass, n_out, n_name, n_conf, n_type);
    }
  }
  if (debug_) draw_detections(bgr_img, armors, frame_count);

  // ⭐⭐ W64：把本帧统计带出去（main 会填进 `FrameDebug::detector`）
  {
    DetectStats st;
    st.n_pass = n_pass;
    st.nms_survivors = nms_survivors;
    st.n_out = static_cast<int>(armors.size());
    for (const auto & a : armors)
      if (a.confidence > st.best_conf) st.best_conf = a.confidence;
    set_last_detect_stats(st);
  }

  return armors;
}

bool YOLOV8::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于分类器的迭代
  // if (name_ok && !confidence_ok) save(armor);

  return name_ok && confidence_ok;
}

bool YOLOV8::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  // 保存异常的图案，用于分类器的迭代
  // if (!name_ok) save(armor);

  return name_ok;
}

ArmorType YOLOV8::get_type(const Armor & armor)
{
  // 英雄、基地只能是大装甲板
  if (armor.name == ArmorName::one || armor.name == ArmorName::base) {
    return ArmorType::big;
  }

  // 工程、哨兵、前哨站只能是小装甲板
  if (
    armor.name == ArmorName::two || armor.name == ArmorName::sentry ||
    armor.name == ArmorName::outpost) {
    return ArmorType::small;
  }

  // 步兵假设为小装甲板
  return ArmorType::small;
}

cv::Point2f YOLOV8::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

cv::Mat YOLOV8::get_pattern(const cv::Mat & bgr_img, const Armor & armor) const
{
  // 延长灯条获得装甲板角点
  // 1.125 = 0.5 * armor_height / lightbar_length = 0.5 * 126mm / 56mm
  auto tl = (armor.points[0] + armor.points[3]) / 2 - (armor.points[3] - armor.points[0]) * 1.125;
  auto bl = (armor.points[0] + armor.points[3]) / 2 + (armor.points[3] - armor.points[0]) * 1.125;
  auto tr = (armor.points[2] + armor.points[1]) / 2 - (armor.points[2] - armor.points[1]) * 1.125;
  auto br = (armor.points[2] + armor.points[1]) / 2 + (armor.points[2] - armor.points[1]) * 1.125;

  auto roi_left = std::max<int>(std::min(tl.x, bl.x), 0);
  auto roi_top = std::max<int>(std::min(tl.y, tr.y), 0);
  auto roi_right = std::min<int>(std::max(tr.x, br.x), bgr_img.cols);
  auto roi_bottom = std::min<int>(std::max(bl.y, br.y), bgr_img.rows);
  auto roi_tl = cv::Point(roi_left, roi_top);
  auto roi_br = cv::Point(roi_right, roi_bottom);
  auto roi = cv::Rect(roi_tl, roi_br);

  // 检查ROI是否有效
  if (roi_left < 0 || roi_top < 0 || roi_right <= roi_left || roi_bottom <= roi_top) {
    // std::cerr << "Invalid ROI: " << roi << std::endl;
    return cv::Mat();  // 返回一个空的Mat对象
  }

  // 检查ROI是否超出图像边界
  if (roi_right > bgr_img.cols || roi_bottom > bgr_img.rows) {
    // std::cerr << "ROI out of image bounds: " << roi << " Image size: " << bgr_img.size()
    //           << std::endl;
    return cv::Mat();  // 返回一个空的Mat对象
  }

  return bgr_img(roi);
}

void YOLOV8::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, armor.pattern);
}

void YOLOV8::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {}", armor.confidence, ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }
  // ⭐⭐⭐ W61：**门控** —— 原来这两种操作**无条件每帧执行**（同济调试残留，绕过 `SinkHub`）：
  //   `resize` ≈ 0.133 ms + `imshow` ≈ 0.70 ms（有 DISPLAY）→ ⚠️ 合计 ~0.83 ms/帧
  //   实测代价见 `utils/debug/l3_gate.hpp` 的注释。
  if (tools::l3_image_wanted()) {
    cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("detection", detection);
  }
}

void YOLOV8::sort_keypoints(std::vector<cv::Point2f> & keypoints)
{
  if (keypoints.size() != 4) {
    std::cout << "beyond 4!!" << std::endl;
    return;
  }

  std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
  std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

  std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });

  std::sort(
    bottom_points.begin(), bottom_points.end(),
    [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });

  keypoints[0] = top_points[0];     // top-left
  keypoints[1] = top_points[1];     // top-right
  keypoints[2] = bottom_points[1];  // bottom-right
  keypoints[3] = bottom_points[0];  // bottom-left
}

std::list<Armor> YOLOV8::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim