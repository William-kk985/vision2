#include "camera.hpp"

#include <stdexcept>

#include "drivers/hikrobot/hikrobot.hpp"
#include "drivers/mindvision/mindvision.hpp"
#include "io/camera/video.hpp"   // ⭐ 录像回放
#include "utils/yaml/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path, bool dump_params)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");

  // ⭐ 录像回放：不需要 exposure_ms，所以放在前面分支
  if (camera_name == "video") {
    auto path = tools::read<std::string>(yaml, "video_path");
    auto speed = yaml["video_speed"] ? yaml["video_speed"].as<double>() : 1.0;
    camera_ = std::make_unique<VideoCamera>(path, speed);
    return;
  }

  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    auto fps = yaml["fps"] ? yaml["fps"].as<double>() : 30.0;

    // ⭐⭐ W90：解析 yaml 的 `camera_params`（任意海康参数，不用开 MVS）
    //   ```yaml
    //   camera_params:
    //     float: { Gamma: 1.0, Sharpness: 50 }
    //     enum:  { BalanceWhiteAuto: 0 }
    //     int:   { Width: 1280 }
    //   ```
    std::vector<CameraParam> extra;
    if (auto cp = yaml["camera_params"]) {
      const std::pair<const char *, CameraParam::Kind> groups[] = {
        {"float", CameraParam::Float}, {"enum", CameraParam::Enum}, {"int", CameraParam::Int}};
      for (const auto & [key, kind] : groups) {
        if (!cp[key] || !cp[key].IsMap()) continue;
        for (const auto & kv : cp[key]) {
          CameraParam p;
          p.kind = kind;
          p.name = kv.first.as<std::string>();
          if (kind == CameraParam::Float) p.fval = kv.second.as<double>();
          else                            p.ival = kv.second.as<int64_t>();
          extra.push_back(p);
        }
      }
      tools::logger()->info(
        "[Camera] yaml 的 camera_params: {} 项（float {}/enum {}/int {}）",
        extra.size(),
        cp["float"] && cp["float"].IsMap() ? cp["float"].size() : 0,
        cp["enum"]  && cp["enum"].IsMap()  ? cp["enum"].size()  : 0,
        cp["int"]   && cp["int"].IsMap()   ? cp["int"].size()   : 0);
    }
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, fps, extra, dump_params);
  }

  else {
    throw std::runtime_error(
      "Unknow camera_name: " + camera_name + "!  (支持: mindvision / hikrobot / video)");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io