#include "camera.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <vector>

// ⭐⭐⭐ W101（原 F7）：**按需 include** —— 没装某厂商 SDK 时它的 `.cpp` 不参与编译，
//   头文件也就不该被引用（否则 `#include` 先失败，比链接失败更难懂）。
//   ⚠️ 宏 `HZMIR_HAS_XXX` 由 `drivers/CMakeLists.txt` **探测 SDK 后定义**。
//   本文件是**装配点** ⇒ 在这里用 `#ifdef` 符合宏规范（允许的 4 处之一）。
#ifdef HZMIR_HAS_HIKROBOT
#  include "drivers/hikrobot/hikrobot.hpp"
#endif
#ifdef HZMIR_HAS_MINDVISION
#  include "drivers/mindvision/mindvision.hpp"
#endif
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

#ifdef HZMIR_HAS_HIKROBOT
  // ⭐⭐⭐ W101（原 F7）：**只在有海康 SDK 时才需要它** ——
  //   返回类型 `CameraParam` 定义在 `drivers/hikrobot/hikrobot.hpp`，
  //   ⚠️ 缺 SDK 时该头不会被 include ⇒ 这里若不保护就是**编译错误**（实测踩到）。
  //   ⭐ `camera_params`（float/enum/int 三组通用参数）**只有海康驱动实现**，
  //     迈德威视/USB 相机不消费它。
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
#endif  // HZMIR_HAS_HIKROBOT
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
    // ⭐⭐⭐ W102：**品牌文件的定位链**（重写过 —— 原来只按 `cam_path` 的父目录找）
    //
    // ⚠️ 原来的 bug：`cands` 只有两条，**都基于 `cam_path` 的父目录**。
    //   而 W102 把"兵种相机配置"并回兵种 yaml 后，`cam_path` **是空的** ⇒
    //   父目录为空 ⇒ 去处变成 CWD 下的 `hikrobot.yaml` / `cameras/hikrobot.yaml`
    //   ⇒ **品牌专配永远加载不到**（静默降级成内置默认）。
    //
    // ⭐ 现在的顺序（从"最贴近配置"到"最宽泛"）：
    //   ① 与 `--camera-config` 同目录 / 其下 cameras/   （显式指定时用）
    //   ② ⭐ **兵种 yaml 的兄弟目录 `cameras/`**
    //      如 `params/robots/infantry.yaml` → `params/cameras/hikrobot.yaml`
    //      ⇒ **不依赖 CWD**（这才是主线路径）
    //   ③ CWD 下的 `params/cameras/` / `cameras/`（兜底）
    std::vector<std::filesystem::path> cands;
    if (!cam_path.empty()) {
      const auto cp = std::filesystem::path(cam_path).parent_path();
      cands.push_back(cp / (camera_name + ".yaml"));
      cands.push_back(cp / "cameras" / (camera_name + ".yaml"));
    }
    if (!config_path.empty()) {                       // ⭐ ② 主线：跟着兵种 yaml 走
      const auto rp = std::filesystem::path(config_path).parent_path();
      cands.push_back(rp / "cameras" / (camera_name + ".yaml"));
      cands.push_back(rp / ".." / "cameras" / (camera_name + ".yaml"));
    }
    cands.push_back(std::filesystem::path("params") / "cameras" / (camera_name + ".yaml"));
    cands.push_back(std::filesystem::path("cameras") / (camera_name + ".yaml"));
    std::filesystem::path found;
    for (const auto & c : cands) {
      if (std::filesystem::exists(c)) { found = c; break; }
    }
    if (!found.empty()) {
      y.vendor = tools::load(found.string());
      // ⭐ W102：规范化后打印（否则 `params/robots/../cameras/x.yaml` 这种不好读）
      tools::logger()->info("[Camera] 品牌专配: {}", found.lexically_normal().string());
    } else {
      std::string tried;
      for (const auto & c : cands) tried += "\n      " + c.lexically_normal().string();
      // ⭐ W102：原来只打 `cands[0]` —— 候选变多后那样不够排查
      tools::logger()->debug("[Camera] 无品牌专配（找过这些路径）：{}", tried);
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
#ifdef HZMIR_HAS_MINDVISION
    auto gamma = y.read<double>("gamma", 0.6);
    auto vid_pid = y.read<std::string>("vid_pid", "");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
#else
    throw std::runtime_error(
      "camera_name=mindvision，但**构建时没找到迈德威视 SDK** ⇒ 该驱动未被编译。\n"
      "  怎么办：① 装 SDK 到 drivers/mindvision/{include,lib/<arch>}/ 后重新 cmake\n"
      "          ② 或把 yaml 的 camera_name 改成 hikrobot / usbcamera\n"
      "          ③ 只跑录像回放则用 --video=<路径>.avi（不需要相机）");
#endif
  } else if (camera_name == "hikrobot") {
    // ⭐⭐⭐ W101（原 F7）：**整块放进 `#ifdef`** —— 缺 SDK 时
    //   `HikRobot` 类型、`CameraParam`、`y.params()` 都不存在，
    //   ⚠️ 只保护 `make_unique` 那一行是不够的（实测：`CameraParam` 未声明）。
#ifdef HZMIR_HAS_HIKROBOT
    auto gain = y.read<double>("gain", 16.0);
    auto vid_pid = y.read<std::string>("vid_pid", "");
    auto fps = y.read<double>("fps", 30.0);

    // ⭐⭐ W90/W92：通用相机参数（`camera_params` 的 float/enum/int 三组）
    auto extra = y.params();
    if (!extra.empty())
      tools::logger()->info("[Camera] camera_params 共 {} 项（相机专配 + 兵种覆盖）", extra.size());

    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, fps, extra, dump_params);
#else
    throw std::runtime_error(
      "camera_name=hikrobot，但**构建时没找到海康 MVS SDK** ⇒ 该驱动未被编译。\n"
      "  怎么办：① 装 MVS 到 drivers/hikrobot/{include,lib/<arch>}/ 后重新 cmake\n"
      "          ② 或把 yaml 的 camera_name 改成 usbcamera\n"
      "          ③ 只跑录像回放则用 --video=<路径>.avi（不需要相机）");
#endif
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
