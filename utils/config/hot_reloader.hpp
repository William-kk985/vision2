/**
 * @file utils/config/hot_reloader.hpp
 * @brief ⭐ 轮子：配置热重载（**你的实现**）
 *
 * **来源**：`Hz_Rm_Vision/src/utils/config/hot_reloader.hpp`（header-only，你已用于 `src/apps/infantry.cpp`）
 * **落位理由**：它一个业务类型都没用（只要 `<yaml-cpp>` + logger）→ 属于 `utils/`
 * **命名空间**：`auto_aim` → `tools`
 *
 * ## ⭐ 移植时修的 2 处
 *
 * | # | 原问题 | 本实现 |
 * |---|---|---|
 * | **B11** | ⭐ **回调顺序与注释不符**：`std::map<std::string, ...>` 是**字典序**，<br>而注释写「按注册顺序依次调用」 | `std::vector<std::pair<name, cb>>` → **真按注册顺序** |
 * | **B12** | ⭐ 每次 `reload()` 都**无条件**触发全部回调（即使文件没变）→<br>在按键/轮询场景下会**反复重配** | 加 **`reload_if_changed()`**（比 mtime，没变就跳过） |
 *
 * ## 错误处理（保留你的原设计）
 * 配置解析失败 → **保留原有配置不变**，只记日志。
 */
#ifndef HZMIR_UTILS_CONFIG_HOT_RELOADER_HPP
#define HZMIR_UTILS_CONFIG_HOT_RELOADER_HPP

#include <sys/stat.h>

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "utils/log/logger.hpp"

namespace tools
{

class HotReloader
{
public:
  using ReloadCallback = std::function<void(const YAML::Node &)>;

  explicit HotReloader(std::string config_path) : config_path_(std::move(config_path)) {}

  /// @brief 注册回调（⭐ B11：**按注册顺序**调用，不再是 map 的字典序）
  void on(std::string name, ReloadCallback cb)
  {
    callbacks_.emplace_back(std::move(name), std::move(cb));
  }

  /// @brief 重新加载并通知全部组件
  /// @return 成功与否；**失败时所有组件保持旧配置**
  bool reload()
  {
    YAML::Node cfg;
    try {
      cfg = YAML::LoadFile(config_path_);
    } catch (const YAML::BadFile &) {
      tools::logger()->error(
        "[HotReload] 打不开配置文件 '{}'（保持旧配置）", config_path_);
      return false;
    } catch (const YAML::ParserException & e) {
      tools::logger()->error(
        "[HotReload] 配置语法错误，第 {} 行: {}（保持旧配置）", e.mark.line, e.msg);
      return false;
    } catch (const YAML::Exception & e) {
      tools::logger()->error("[HotReload] YAML 错误: {}（保持旧配置）", e.what());
      return false;
    } catch (const std::exception & e) {
      tools::logger()->error("[HotReload] 未知错误: {}（保持旧配置）", e.what());
      return false;
    }

    // 解析成功才逐个通知；单个组件失败不影响其他组件
    int ok = 0, bad = 0;
    for (auto & [name, cb] : callbacks_) {
      try {
        cb(cfg);
        tools::logger()->info("[HotReload] 已更新: {}", name);
        ++ok;
      } catch (const std::exception & e) {
        tools::logger()->error("[HotReload] 组件 '{}' 更新失败: {}", name, e.what());
        ++bad;
      }
    }
    ++reload_count_;
    last_mtime_ = mtime();
    tools::logger()->warn("[HotReload] 完成：{} 个组件更新，{} 个失败", ok, bad);
    return true;
  }

  /// @brief ⭐ B12：只在文件 mtime 变化时才 reload
  /// @return 是否真的执行了 reload
  bool reload_if_changed()
  {
    const int64_t m = mtime();
    if (m == 0) {
      tools::logger()->warn("[HotReload] 读不到 {} 的 mtime", config_path_);
      return false;
    }
    if (m == last_mtime_) {
      tools::logger()->info("[HotReload] 配置未变化，跳过");
      return false;
    }
    return reload();
  }

  /// @brief 文件 mtime（秒；0 = 读不到）
  int64_t mtime() const
  {
    struct stat st{};
    if (::stat(config_path_.c_str(), &st) != 0) return 0;
    return static_cast<int64_t>(st.st_mtime);
  }

  size_t size() const { return callbacks_.size(); }
  const std::string & path() const { return config_path_; }
  int reload_count() const { return reload_count_; }

private:
  std::string config_path_;
  std::vector<std::pair<std::string, ReloadCallback>> callbacks_;   // ⭐ B11
  int64_t last_mtime_ = -1;
  int reload_count_ = 0;
};

}  // namespace tools

#endif  // HZMIR_UTILS_CONFIG_HOT_RELOADER_HPP
