#include "core/auto_aim/target/target_plugins_factory.hpp"

#include <algorithm>
#include <cctype>

#include "utils/log/logger.hpp"

namespace auto_aim
{

namespace
{
std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

/// ⚠️ 非法值**不静默回退** —— 打 warn 并明确用哪个（D11 的教训）
std::string pick(
  const std::string & raw, const std::vector<std::string> & allowed, const std::string & fallback,
  const char * key)
{
  if (raw.empty()) return fallback;
  const auto v = lower(raw);
  if (std::find(allowed.begin(), allowed.end(), v) != allowed.end()) return v;
  tools::logger()->warn(
    "[TargetPlugins] `{}: {}` 不是合法值（可选 {}）→ **用默认 `{}`（= 同济行为）**", key, raw,
    [&] {
      std::string s;
      for (size_t i = 0; i < allowed.size(); ++i) s += (i ? "/" : "") + allowed[i];
      return s;
    }(),
    fallback);
  return fallback;
}
}  // namespace

std::string parse_plugin_name(const YAML::Node & yaml, const char * key, const char * fallback)
{
  const auto node = yaml["target_plugins"];
  if (!node || !node[key]) return fallback;
  return node[key].as<std::string>();
}

void apply_target_plugins_from_yaml(Target & target, const YAML::Node & yaml)
{
  const auto node = yaml["target_plugins"];
  const bool has = static_cast<bool>(node);

  // ── ① 角速度估计 ──
  const auto w_raw = has ? pick(node["angular_velocity"] ? node["angular_velocity"].as<std::string>() : "",
                                {"ekf_state", "ekf", "visual_diff", "imu_fusion"}, "ekf_state",
                                "angular_velocity")
                         : std::string("ekf_state");
  if (w_raw == "ekf") {
    target.set_angular_velocity_estimator(std::make_unique<EkfOnly>());
  } else if (w_raw == "visual_diff") {
    target.set_angular_velocity_estimator(std::make_unique<VisualDiff>());
  } else if (w_raw == "imu_fusion") {
    const double alpha = (node && node["imu_alpha"]) ? node["imu_alpha"].as<double>() : 0.7;
    target.set_angular_velocity_estimator(std::make_unique<ImuFusion>(alpha));
  } else {
    target.set_angular_velocity_estimator(std::make_unique<EkfStateOnly>());   // = 同济
  }

  // ── ② 过程噪声 ──
  const auto q_raw = has ? pick(node["process_noise"] ? node["process_noise"].as<std::string>() : "",
                                {"fixed", "nis"}, "fixed", "process_noise")
                         : std::string("fixed");
  if (q_raw == "nis") {
    target.set_process_noise_adapter(std::make_unique<NisBasedAdapter>());
  } else {
    target.set_process_noise_adapter(std::make_unique<FixedNoise>());   // = 同济
  }

  // ── ③ 观测滤波 ──
  const auto m_raw = has ? pick(node["measurement"] ? node["measurement"].as<std::string>() : "",
                                {"passthrough", "median"}, "passthrough", "measurement")
                         : std::string("passthrough");
  if (m_raw == "median") {
    // ⚠️ `MedianFilter` 的窗口是 `static constexpr WINDOW_SIZE = 5`（与原实现一致），无构造参数
    target.set_measurement_filter(std::make_unique<MedianFilter>());
  } else {
    target.set_measurement_filter(std::make_unique<PassthroughFilter>());   // = 同济
  }

  if (has) {
    tools::logger()->info(
      "[TargetPlugins] ⭐ 已从 yaml 装配：angular_velocity={} process_noise={} measurement={}",
      w_raw, q_raw, m_raw);
  }
}

}  // namespace auto_aim
