#include "utils/system/paths.hpp"   // ⭐ W102：落图路径
#include "yolov5.hpp"
#include "../det_stats.hpp"   // ⭐ W63：检测统计开关
#include "utils/debug/l3_gate.hpp"   // ⭐ W61：全局 L3 门控

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>

#include "utils/ov/device.hpp"   // ⭐ W28：device 回退
#include "utils/debug/img_tools.hpp"
#include "utils/log/logger.hpp"

namespace auto_aim
{
YOLOV5::YOLOV5(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolov5_model_path"].as<std::string>();
  // ⭐ W28：解析设备（不可用时回退 CPU，而不是抛异常）
  //   同济原句：`device_ = yaml["device"].as<std::string>();`
  device_ = tools::resolve_device(
    core_, yaml["device"] ? yaml["device"].as<std::string>() : "");
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  // ⭐⭐ W76：阈值可配（不写 = 同济原值 0.7 / 0.3）
  if (yaml["score_threshold"]) score_threshold_ = yaml["score_threshold"].as<float>();
  if (yaml["nms_threshold"]) nms_threshold_ = yaml["nms_threshold"].as<float>();
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  // ⭐⭐⭐ W102：**路径走 `paths`（`output/images`），不再硬编码仓库顶层 `imgs/`**
  //   ⚠️ 原来 `create_directory("imgs")` 在**构造时**就执行 ⇒ 跑一次就在仓库根建个 `imgs/`
  //     （实测确实被建出来过，空的）。改成惰性（见 `save()`）。
  save_path_ = tools::paths::images();
  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  // ⭐ W83：`performance_mode(LATENCY)` **已设**（见下面 compile_model）；
  //   原来这里留着一句 TODO 会让人以为没设 —— 删掉避免误导。
  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

DetectorResult YOLOV5::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return DetectorResult{};   // ⭐ W98
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

  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // preproces
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  // infer
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

    return parse(scale, output, raw_img, frame_count);   // ⭐ W98：parse 直接带出 dbg
}

DetectorResult YOLOV5::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // for each row: xywh + classess
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  // ⭐⭐⭐ W62：**逐帧统计各步过滤** —— 解决「识别不出来时不知道卡在哪」
  //   原来 objectness/not_armor/置信度/类型 四种失败【都没有日志】、
  //   `armor_count` 又是过滤后的 → 终端和 CSV 都分不出卡在哪一步。
  int n_pass = 0, n_name = 0, n_conf = 0, n_type = 0;   // ⭐ 只统计 objectness 通过后的候选
  // ⭐⭐ W68：记录本帧 objectness **分布** —— 只看"最高多少"信息量太少，
  //   看"有多少 anchor 接近阈值"才知道离能用还有多远。
  //   ⚠️ 不再记录"最像的 anchor 的类别/颜色"：那两路输出**只有在 objectness 高时才有意义**，
  //      对 objectness≈0.25 的 anchor 打印它们纯属误导（用户实测反馈）。
  double max_obj = 0;
  int n_over_05 = 0, n_over_03 = 0, n_over_01 = 0;
  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);

      if (score > max_obj) max_obj = score;   // ⭐ W68：便宜（一次比较）
      if (score > 0.5) ++n_over_05;
      if (score > 0.3) ++n_over_03;
      if (score > 0.1) ++n_over_01;
      if (score < score_threshold_) continue;   // ⭐ 大部分 anchor 都在这（不是装甲板）
      ++n_pass;                                  // ⭐ 通过 objectness = "像装甲板"的候选

    std::vector<cv::Point2f> armor_key_points;

    //颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);     //color
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  //num
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    _class_id = class_id.x;
    _color_id = color_id.x;

    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

    for (int i = 1; i < armor_key_points.size(); i++) {
      if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);
  const int nms_survivors = static_cast<int>(indices.size());   // ⭐ W64：NMS 存活数（原来算完就扔）

  std::list<Armor> armors;
  for (const auto & i : indices) {
    if (use_roi_) {
      armors.emplace_back(
        color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();) {
    if (it->name == ArmorName::not_armor) { ++n_name; it = armors.erase(it); continue; }
    if (it->confidence <= min_confidence_) { ++n_conf; it = armors.erase(it); continue; }
    if (!check_name(*it)) {
      ++n_conf;
    }

    if (!check_type(*it)) {
      ++n_type;
      it = armors.erase(it);
      continue;
    }
    // 使用传统方法二次矫正角点
    if (use_traditional_) detector_.detect(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  // ⭐⭐⭐ W62/W68：**一帧一行汇总** —— 一眼看出卡在哪一步
  //   ⚠️ `n_pass` 才是"像装甲板的候选数"；anchor 总数（25200）没意义、不打。
  //   ⭐ W68：没候选时只报 **objectness 分布**（离阈值多远）；
  //     不再报"最像的 anchor 的类别/颜色"—— 那两路输出**只在 objectness 高时才有意义**，
  //     对 objectness≈0.2 的 anchor 打印它们纯属误导（用户实测反馈）。
  if (auto_aim::det_stats_enabled() && n_pass == 0) {
    static int no_cand_count = 0;
    if (++no_cand_count % 60 == 1)   // 节流：每 60 帧（约 0.6 秒）报一次
      // ⭐⭐⭐ W106：**改用 `logger()->debug`（运行期可见）而不是 `LOG_YOLO`（编译期宏）**
      //   ⚠️ W99 我把它塞进了宏 —— **那是做错了**：
      //     它**每 60 帧才一条**（上一行节流），是【概览级】信息，不是"逐帧细节"。
      //   ⭐ 走 logger ⇒ **不用重编**（热键 `d` 调级别 / `--log-off` 静音），
      //     这正是"大概情况运行期看、具体细节用宏"的分工。
      tools::logger()->debug(
        "[yolov5] 本帧无候选：objectness 峰值 {:.3f} < 阈值 {:.2f}"
        "（采样 {} 个 anchor；>0.3 有 {} 个、>0.1 有 {} 个）",
        max_obj, score_threshold_, output.rows, n_over_03, n_over_01);
  }
  if (auto_aim::det_stats_enabled() && n_pass > 0) {
    static int last_n_out = -1;
    static int since_log = 0;
    const int n_out = static_cast<int>(armors.size());
    ++since_log;
    // ⭐⭐⭐ W106：**改成「硬性限频」** —— 原来只在"输出数没变"时节流，
    //   ⚠️ 而输出数**一抖动**（0→1→0→1）就**每帧都打**（实测 687 帧打了 144 条）。
    //   ⭐ 现在：**至少隔 30 帧才打一条**；但如果输出数**变了**，可以提前打
    //     （最多提前到 10 帧 —— 变化要让人看见，但不能变成刷屏）。
    const bool changed = (n_out != last_n_out);
    if (since_log < (changed ? 10 : 30)) {
      // 限频中，跳过
    } else {
      since_log = 0;
      last_n_out = n_out;
      if (n_out == 0)
        // ⭐ W106：同上 —— 这条**每 30 帧才一条**（上面节流），概览级 ⇒ 用 logger
        tools::logger()->debug(
          "[{}] objectness 通过 {} 个候选 → 全被滤掉：not_armor {} / 置信度 {} / "
          "类型不符 {} 最终 0 个装甲板",
          "yolov5", n_pass, n_name, n_conf, n_type);
      else
        tools::logger()->debug(
          "[{}] objectness 通过 {} → 输出 {}（滤掉 not_armor {} / conf {} / type {}）", "yolov5",
          n_pass, n_out, n_name, n_conf, n_type);
    }
  }
  if (debug_) draw_detections(bgr_img, armors, frame_count);

  // ⭐⭐ W98：统计**随返回值带出**（原来写进全局 `last_detect_stats()`，
  //   主循环再回头读 —— 那条旁路正是 `best_confidence`/`nms_survivors`
  //   长期为 0 的原因，W64 才补上。现在结构上不可能忘。）
  DetectorResult r;
  r.dbg.n_pass = n_pass;
  // ⭐ W106：**objectness 峰值 + 门槛进 `FrameDebug`** ⇒ CSV / PlotJuggler / 窗口
  //   都能看（**不用重编**）—— 这是"概览"的另一条路（比日志更适合看趋势）。
  r.dbg.objectness_peak = max_obj;
  r.dbg.score_threshold = score_threshold_;
  r.dbg.nms_survivors = nms_survivors;
  r.dbg.armor_count = static_cast<int>(armors.size());
  for (const auto & a : armors)
    if (a.confidence > r.dbg.best_confidence) r.dbg.best_confidence = a.confidence;
  r.armors = std::move(armors);
  return r;
}

bool YOLOV5::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于神经网络的迭代
  // if (name_ok && !confidence_ok) save(armor);

  return name_ok && confidence_ok;
}

bool YOLOV5::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  // 保存异常的图案，用于神经网络的迭代
  // if (!name_ok) save(armor);

  return name_ok;
}

cv::Point2f YOLOV5::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLOV5::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
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
    tools::register_l3_window("detection");   // ⭐ W77：登记，关闸时统一销毁
  }
}

void YOLOV5::save(const Armor & armor) const
{
  // ⭐ W102：惰性建目录（原来构造时就建 ⇒ 不落图也多个空目录）
  static bool dir_ready = false;
  if (!dir_ready) {
    tools::paths::ensure_dir(save_path_);
    dir_ready = true;
  }
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, tmp_img_);
}

double YOLOV5::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

std::list<Armor> YOLOV5::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count).armors;   // ⭐ W98：postprocess 只关心结果
}

}  // namespace auto_aim