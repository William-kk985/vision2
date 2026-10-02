/**
 * @file utils/debug/plotjuggler_sink.hpp
 * @brief ⭐ PlotJuggler sink —— 把 FrameDebug 打成 JSON 发 UDP（127.0.0.1:9870）
 *
 * ⚠️ **必须带 `timestamp` 字段**，否则 PlotJuggler 落不了时间轴
 *    （本仓库原本的 plotter 输出**没有** timestamp —— `05`/E5）
 *
 * ## ⭐⭐ W80：JSON 拼装搬到 worker 线程（自瞄线程只剩入队）
 * 原来 `on_frame()` 在**自瞄线程**里做：
 *   `std::ostringstream` 拼 ~18 个字段的 JSON + `::sendto()`  ⇒ 实测 **~1 µs/帧**
 * ⇒ 现在和 `CsvSink` 一个模式：**自瞄线程只做 400 B POD 拷贝 + 入队（~250 ns）**，
 *   格式化 + `sendto` 都在 worker。
 *
 * ⚠️ **`timestamp` 必须在【入队时】取**（`on_frame` 里）—— 否则取到的是 worker 的时间，
 *    曲线会滞后于真实帧时刻。
 *
 * ⭐ 队列**有界**（默认 64 条）→ 满了**丢最旧的**（实时曲线丢几帧无所谓；
 *    绝不无限增长撑爆内存）。丢帧计数可查。
 */
#ifndef HZMIR_UTILS_DEBUG_PLOTJUGGLER_SINK_HPP
#define HZMIR_UTILS_DEBUG_PLOTJUGGLER_SINK_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "debug_sink.hpp"

struct sockaddr_in;   // 前置声明（全局作用域；避免把系统头带进头文件）

namespace tools
{

class PlotJugglerSink : public IDebugSink
{
public:
  explicit PlotJugglerSink(std::string host = "127.0.0.1", uint16_t port = 9870, bool enabled = true);
  ~PlotJugglerSink() override;

  const char * name() const override { return "plotjuggler"; }
  void on_frame(const auto_aim::FrameDebug & d) override;

  /// @brief 直接发一段自定义 JSON（兼容旧用法）
  void send(const std::string & json);

  /// @brief 自瞄线程丢帧数（队列满时；正常应为 0）
  int64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
  /// ⭐ 入队的单元：**入队时刻的时间戳** + 400 B POD 的 `FrameDebug`
  struct Queued
  {
    double ts = 0;
    auto_aim::FrameDebug d{};
  };

  void worker();   // ⭐ W80：JSON 拼装 + sendto 都在这里

  int fd_ = -1;
  ::sockaddr_in * dest_ = nullptr;
  bool enabled_ = true;
  std::chrono::steady_clock::time_point t0_;

  // ⭐ W80：自瞄线程 ⇄ worker 的交接
  static constexpr size_t kMaxQueue = 64;   // 有界（~26 KB；满了丢旧帧）
  std::deque<Queued> q_;
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  std::thread th_;
  std::atomic<bool> quit_{false};
  std::atomic<int64_t> dropped_{0};
};

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_PLOTJUGGLER_SINK_HPP
