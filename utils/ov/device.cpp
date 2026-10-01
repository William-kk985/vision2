#include "utils/ov/device.hpp"

#include <algorithm>
#include <stdexcept>

#include "utils/log/logger.hpp"

namespace tools
{

namespace
{
bool g_strict = false;

/// "GPU" 应匹配 "GPU" 或 "GPU.0" / "GPU.1"
bool matches(const std::string & avail, const std::string & want)
{
  if (avail == want) return true;
  return avail.size() > want.size() && avail.compare(0, want.size(), want) == 0 &&
         avail[want.size()] == '.';
}
}  // namespace

void set_strict_device(bool on) { g_strict = on; }
bool strict_device() { return g_strict; }

std::vector<std::string> available_devices(ov::Core & core)
{
  try {
    return core.get_available_devices();
  } catch (const std::exception & e) {
    tools::logger()->warn("[Device] get_available_devices 失败: {}", e.what());
    return {"CPU"};
  }
}

std::string describe_devices(ov::Core & core)
{
  const auto devs = available_devices(core);
  std::string s;
  for (size_t i = 0; i < devs.size(); ++i) {
    if (i) s += ";";
    s += devs[i];
  }
  return s;
}

std::string resolve_device(ov::Core & core, const std::string & requested)
{
  std::string want = requested;
  // yaml 没写 / 写空 → CPU（与同济 yolo11_buff 的硬编码一致）
  if (want.empty()) want = "CPU";

  // AUTO / MULTI / HETERO 交给 OpenVINO 自己调度，不干预
  if (want == "AUTO" || want.rfind("MULTI:", 0) == 0 || want.rfind("HETERO:", 0) == 0 ||
      want.rfind("AUTO:", 0) == 0) {
    tools::logger()->info("[Device] 请求 '{}' → 交给 OpenVINO 调度", want);
    return want;
  }

  const auto devs = available_devices(core);

  const bool ok = std::any_of(devs.begin(), devs.end(),
                              [&](const std::string & d) { return matches(d, want); });
  if (ok) {
    tools::logger()->info("[Device] 使用 '{}'（可用: {}）", want, describe_devices(core));
    return want;
  }

  // ⚠️ 不可用
  if (g_strict) {
    // 同济原行为：抛异常（进程会 abort，但错误信息更明确）
    throw std::runtime_error(
      "[Device] 请求的设备 '" + want + "' 不可用（可用: " + describe_devices(core) +
      "）；严格模式已开启 → 拒绝回退");
  }

  tools::logger()->warn(
    "[Device] ⚠️ 请求的设备 '{}' **不可用**（可用: {}）→ **回退 CPU**。"
    "若要恢复同济的严格行为：tools::set_strict_device(true)",
    want, describe_devices(core));
  return "CPU";
}

}  // namespace tools
