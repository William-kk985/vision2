#include "camera.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>

#include "drivers/hikrobot/hikrobot.hpp"
#include "drivers/mindvision/mindvision.hpp"
#include "io/camera/video.hpp"   // ⭐ 录像回放
#include "utils/log/logger.hpp"
#include "utils/yaml/yaml.hpp"

namespace io
{

namespace
{
/// ⭐⭐ W92：**按优先级读一个键**：兵种 yaml > 相机专配 > 空
///
/// 这样「4 兵种共用的相机参数」只写在 `params/camera.yaml` 一处；
/// 某个兵种要特殊值（如 sentry 曝光 0.8ms、uav 用 mindvision），
/// 就在自己的 yaml 里写同名键覆盖即可。
struct MergedYaml
{
  YAML::Node robot;             ///< 兵种 yaml（`params/<兵种>.yaml`）
  YAML::Node camera;            ///< 相机专配（`params/camera.yaml`），可能为空

  YAML::Node get(const char * key) const
  {
    if (robot[key]) return robot[key];              // ⭐ 兵种覆盖优先
    if (camera && camera[key]) return camera[key];  // ⭐ 其次共用配置
    return YAML::Node();
  }

  template <typename T>
  T read(const char * key, const T & fallback) const
  {
    auto n = get(key);
    return n ? n.as<T>() : fallback;
  }

  /// @brief 合并两组 `camera_params`（同名时【兵种】优先）
  std::vector<CameraParam> params() const
  {
    std::vector<CameraParam> out;
    const std::pair<const char *, CameraParam::Kind> groups[] = {
      {"float", CameraParam::Float}, {"enum", CameraParam::Enum}, {"int", CameraParam::Int}};

    for (const auto & [key, kind] : groups) {
      // ⭐ 先收 camera.yaml 的，再让兵种 yaml 覆盖同名项
      for (const YAML::Node * src : {&camera, &robot}) {
        if (!*src || !(*src)["camera_params"] || !(*src)["camera_params"][key]) continue;
        for (const auto & kv : (*src)["camera_params"][key]) {
          const std::string name = kv.first.as<std::string>();
          auto it = std::find_if(out.begin(), out.end(), [&](const CameraParam & p) {
            return p.name == name && p.kind == kind;
          });
          CameraParam p;
          p.kind = kind;
          p.name = name;
          if (kind == CameraParam::Float) p.fval = kv.second.as<double>();
          else                            p.ival = kv.second.as<int64_t>();
          if (it != out.end()) *it = p;   // ⭐ 覆盖（robot 后遍历，自然胜出）
          else                 out.push_back(p);
        }
      }
    }
    return out;
  }
};
}  // namespace

Camera::Camera(
  const std::string & config_path, bool dump_params, const std::string & camera_config)
{
  MergedYaml y;
  y.robot = tools::load(config_path);

  // ⭐⭐ W92：相机专配（默认 `params/camera.yaml`）—— 4 兵种共用，不用每份都改
  if (!camera_config.empty() && std::filesystem::exists(camera_config)) {
    y.camera = tools::load(camera_config);
    tools::logger()->info("[Camera] 相机专配: {}（兵种 yaml 可覆盖同名项）", camera_config);
  } else if (!camera_config.empty()) {
    tools::logger()->debug("[Camera] 无相机专配 {} → 只用兵种 yaml 的相机段", camera_config);
  }

  auto camera_name = y.read<std::string>("camera_name", "hikrobot");

  // ⭐ 录像回放：不需要曝光，放在前面分支
  if (camera_name == "video") {
    auto path = y.read<std::string>("video_path", "");
    auto speed = y.read<double>("video_speed", 1.0);
    camera_ = std::make_unique<VideoCamera>(path, speed);
    return;
  }

  auto exposure_ms = y.read<double>("exposure_ms", 2.0);

  if (camera_name == "mindvision") {
    auto gamma = y.read<double>("gamma", 0.6);
    auto vid_pid = y.read<std::string>("vid_pid", "");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  } else if (camera_name == "hikrobot") {
    auto gain = y.read<double>("gain", 16.0);
    auto vid_pid = y.read<std::string>("vid_pid", "");
    auto fps = y.read<double>("fps", 30.0);

    // ⭐⭐ W90/W92：通用相机参数（`camera_params` 的 float/enum/int 三组）
    auto extra = y.params();
    if (!extra.empty())
      tools::logger()->info("[Camera] camera_params 共 {} 项（相机专配 + 兵种覆盖）", extra.size());

    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, fps, extra, dump_params);
  } else {
    throw std::runtime_error(
      "Unknow camera_name: " + camera_name + "!  (支持: mindvision / hikrobot / video)");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
