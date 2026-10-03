/**
 * @file utils/debug/debug_sink.hpp
 * @brief ⭐ Debug 数据面：`IDebugSink` 接口 + `SinkHub`（热插拔）
 *
 * ## 「像节点一样」的原理（doc 09 §14.7）
 * 哈工程的 `*Debug.msg` → 话题 → foxglove，原理不是 msg 也不是 ROS，是 pub/sub 三条性质：
 *   ① 生产者不知道消费者   → 角色只调 `hub.on_frame(...)`
 *   ② 消费者生命周期独立   → `SinkHub::add()/remove()` **热插拔**
 *   ③ 异步                 → 每个 sink 自带线程（落盘/编码都在线程里）
 *
 * ⭐ **关闭一个 sink = 从 sinks_ 里移除** → 后续 `on_frame()` **路径上根本没有它** → 零成本
 *    （不是"进了 sink 再 if (enabled)"）
 *
 * ## ⚠️ 记录在案的纪律例外
 * 本文件 include `core/debug.hpp` / `core/types.hpp`。
 *   理由：sink 的**接口签名必须知道数据结构**，而这两个头是**纯数据契约**（零业务逻辑）。
 *   同类先例：`utils/ekf/` 允许使用 Eigen。
 *   **仍然禁止**：`utils/` 依赖 `core/` 的**业务类**（如 `auto_aim::Tracker`）。
 */
#ifndef HZMIR_UTILS_DEBUG_DEBUG_SINK_HPP
#define HZMIR_UTILS_DEBUG_DEBUG_SINK_HPP

#include <algorithm>
#include <memory>
#include <atomic>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/opencv.hpp>

#include "core/debug_node.hpp"
#include "utils/debug/l3_gate.hpp"   // ⭐ W61：全局 L3 门控
#include "utils/log/logger.hpp"   // ⭐ W61：全局 L3 门控

namespace tools
{

class IDebugSink
{
public:
  virtual ~IDebugSink() = default;

  virtual const char * name() const = 0;

  /// ⭐ L3 门控：是否有 sink 真的在乎图像？
  /// 主循环据此决定**要不要花钱构造 overlay**（不构造 → 零成本）
  virtual bool wants_image() const { return false; }

  /// L0 + L1：每帧一次
  virtual void on_frame(const auto_aim::FrameDebug & d) = 0;

  /// L2：多帧序列（曲线），按 key 分流
  virtual void on_series(std::string_view key, int64_t t_us, double v) { (void)key; (void)t_us; (void)v; }

  /// L3：昂贵（存图/显示）—— ⭐ **纯运行期门控**：挂了 `wants_image()==true` 的 sink 才会被调用
  virtual void on_image(std::string_view tag, const cv::Mat & img, int64_t t_us)
  {
    (void)tag; (void)img; (void)t_us;
  }
};

/// @brief "话题总线"：广播给所有 sink，支持运行期热插拔
class SinkHub
{
public:
  void add(std::shared_ptr<IDebugSink> s)
  {
    if (!s) return;
    std::unique_lock lk(mtx_);
    sinks_.push_back(std::move(s));
      n_.store(sinks_.size(), std::memory_order_release);   // ⭐ W61：原子镜像
      refresh_l3_gate();                                     // ⭐ W61：同步全局门控
  }

  bool remove(std::string_view name)
  {
    std::unique_lock lk(mtx_);
    auto it = std::find_if(sinks_.begin(), sinks_.end(),
                           [&](const auto & s) { return s->name() == name; });
    if (it == sinks_.end()) return false;
    sinks_.erase(it);
      n_.store(sinks_.size(), std::memory_order_release);   // ⭐ W61
      refresh_l3_gate();                                     // ⭐ W61
    return true;
  }

  bool has(std::string_view name) const
  {
    std::shared_lock lk(mtx_);
    return std::any_of(sinks_.begin(), sinks_.end(),
                       [&](const auto & s) { return s->name() == name; });
  }

  /// @brief 有则删、无则加（按键用）
  bool toggle(std::shared_ptr<IDebugSink> s)
  {
    if (has(s->name())) { remove(s->name()); return false; }
    add(std::move(s));
    return true;
  }

  size_t size() const { std::shared_lock lk(mtx_); return sinks_.size(); }

  /// ⭐ 是否有任意 sink 需要图像（L3 门控）—— 热路径每帧问一次，成本忽略
  bool wants_image() const
  {
    if (n_.load(std::memory_order_acquire) == 0) return false;   // ⭐ W61 快路径
    std::shared_lock lk(mtx_);
    for (const auto & s : sinks_)
      if (s->wants_image()) return true;
    return false;
  }

  void on_frame(const auto_aim::FrameDebug & d)
  {
    if (n_.load(std::memory_order_acquire) == 0) return;   // ⭐ W61 快路径
    std::shared_lock lk(mtx_);          // 读多写少
    for (auto & s : sinks_) s->on_frame(d);
  }
  void on_series(std::string_view k, int64_t t, double v)
  {
    if (n_.load(std::memory_order_acquire) == 0) return;   // ⭐ W61 快路径
    std::shared_lock lk(mtx_);
    for (auto & s : sinks_) s->on_series(k, t, v);
  }
  void on_image(std::string_view tag, const cv::Mat & img, int64_t t_us)
  {
    if (n_.load(std::memory_order_acquire) == 0) return;   // ⭐ W61 快路径
    std::shared_lock lk(mtx_);
    for (auto & s : sinks_) s->on_image(tag, img, t_us);
  }

private:
  /// ⭐ W61：把「有没有 sink 要图」同步到全局门控（供 detector/yolo 这类深层代码查）
  void refresh_l3_gate()
    {
      bool want = false;
      for (const auto & s : sinks_)
        if (s->wants_image()) { want = true; break; }
      const bool was = tools::l3_image_wanted();
      tools::set_l3_image_wanted(want);

      // ⭐⭐⭐ W77：门控**由开转关**时，销毁 detector/yolo 自己开的窗口
      //   （`detection` / `binary_img`）—— 它们**不归 `WindowSink` 管**，
      //   没人销毁就会一直留在屏幕上（用户实测：按 1 后 `detection` 窗口没掉）。
      //   `WindowSink` 自己的窗口由它的析构负责（W76）。
      if (was && !want && !tools::l3_windows().empty()) {
        for (const auto & n : tools::l3_windows()) {
          try {
            cv::destroyWindow(n);
          } catch (const std::exception & e) {
            tools::logger()->warn("[L3] 销毁窗口 '{}' 失败: {}", n, e.what());
          }
        }
        cv::waitKey(1);   // OpenCV 的销毁是排队式的，要 pump 一次事件循环
      }
    }

  mutable std::shared_mutex mtx_;
  std::vector<std::shared_ptr<IDebugSink>> sinks_;
  std::atomic<size_t> n_{0};   // ⭐ W61：sink 数量的原子镜像（无 sink 时免锁）
};

/// @brief 什么都不做的 sink（比赛用：零开销）
class NullSink : public IDebugSink
{
public:
  const char * name() const override { return "null"; }
  void on_frame(const auto_aim::FrameDebug &) override {}
};

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_DEBUG_SINK_HPP
