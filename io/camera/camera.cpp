#include "camera.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <vector>

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
/// 相机配置的定位链：`--camera-config` CLI > 兵种 yaml 的 `camera_config:` 键
/// > 品牌默认 `params/cameras/<品牌>.yaml` > 内置默认。
/// 某个兵种要特殊值，就在 `params/cameras/<兵种>.yaml` 里写（或直接在兵种 yaml 覆盖）。
struct MergedYaml
{
  YAML::Node robot;    ///< 兵种 yaml（`params/<兵种>.yaml`）—— 优先级最高
  YAML::Node camera;   ///< 兵种相机配置（`params/cameras/<兵种>.yaml`）
  YAML::Node vendor;   ///< ⭐ 品牌专配（`params/cameras/<品牌>.yaml`）—— 优先级最低

  YAML::Node get(const char * key) const
  {
    if (robot[key]) return robot[key];               // ⭐ ① 兵种覆盖
    if (camera && camera[key]) return camera[key];   // ⭐ ② 本机相机配置
    if (vendor && vendor[key]) return vendor[key];   // ⭐ ③ 品牌默认
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
      // ⭐ 从低优先级到高优先级遍历 ⇒ 后面的自然覆盖前面的
      for (const YAML::Node * src : {&vendor, &camera, &robot}) {
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

  // ⭐⭐ W94：**相机配置的定位链**（不再有全局的 `params/camera.yaml`）：
  //     `--camera-config=<path>` CLI（显式给才有）
  //        ↓ 否则
  //     兵种 yaml 的 `camera_config:` 键（如 `params/cameras/infantry.yaml`）
  //        ↓ 该文件的 `camera_name` 再自动带出
  //     品牌默认 `params/cameras/<品牌>.yaml`（如 hikrobot.yaml / mindvision.yaml）
  //   优先级（低→高）：品牌默认 < 兵种相机配置 < 兵种 yaml 里同名的相机键
  std::string cam_path = camera_config;          // ⭐ CLI 优先
  if (cam_path.empty() && y.robot["camera_config"]) {
    cam_path = y.robot["camera_config"].as<std::string>();
  }

  if (!cam_path.empty() && std::filesystem::exists(cam_path)) {
    y.camera = tools::load(cam_path);
    tools::logger()->info("[Camera] 相机配置: {}", cam_path);
  } else if (!cam_path.empty()) {
    tools::logger()->warn(
      "[Camera] ⚠️ 兵种 yaml 指定的 camera_config 不存在: {} → 只用品牌默认 + 内置默认",
      cam_path);
  }

  // ⭐⭐ W93：**按品牌分文件** —— `params/cameras/<camera_name>.yaml`
  //   放"这个品牌特有的东西"（如海康的 PixelFormat、迈德威视的 gamma），
  //   ⇒ 换相机品牌时不用动兵种 yaml。
  //   优先级最低（可被上面两层覆盖）。
  auto camera_name = y.read<std::string>("camera_name", "hikrobot");
  {
    // ⭐ 品牌文件查找顺序（先近后远）：
    //   ① 与 cam_path **同目录**的 `<品牌>.yaml`
    //      （如 `params/cameras/infantry.yaml` → `params/cameras/hikrobot.yaml`）
    //   ② `<cam_path.parent>/cameras/<品牌>.yaml`
    //      （如 `params/cameras/infantry.yaml` → `params/cameras/hikrobot.yaml`）
    //   ⚠️ 修复：原来只试 ②，当 cam_path 已在 `cameras/` 下时会变成
    //      `params/cameras/cameras/hikrobot.yaml`（多一层，永远找不到）。
    const auto parent = std::filesystem::path(cam_path).parent_path();
    std::vector<std::filesystem::path> cands = {
      parent / (camera_name + ".yaml"),
      parent / "cameras" / (camera_name + ".yaml"),
    };
    std::filesystem::path found;
    for (const auto & c : cands) {
      if (std::filesystem::exists(c)) { found = c; break; }
    }
    if (!found.empty()) {
      y.vendor = tools::load(found.string());
      tools::logger()->info("[Camera] 品牌专配: {}", found.string());
    } else {
      tools::logger()->debug("[Camera] 无品牌专配（找过 {}）", cands[0].string());
    }
  }

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
