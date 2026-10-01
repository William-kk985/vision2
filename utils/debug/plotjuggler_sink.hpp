/**
 * @file utils/debug/plotjuggler_sink.hpp
 * @brief ⭐ PlotJuggler sink —— 把 FrameDebug 打成 JSON 发 UDP（127.0.0.1:9870）
 *
 * ⚠️ **必须带 `timestamp` 字段**，否则 PlotJuggler 落不了时间轴
 *    （本仓库原本的 plotter 输出**没有** timestamp —— `05`/E5）
 */
#ifndef HZMIR_UTILS_DEBUG_PLOTJUGGLER_SINK_HPP
#define HZMIR_UTILS_DEBUG_PLOTJUGGLER_SINK_HPP

#include <chrono>
#include <cstdint>
#include <string>

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

private:
  int fd_ = -1;
  ::sockaddr_in * dest_ = nullptr;
  bool enabled_ = true;
  std::chrono::steady_clock::time_point t0_;
};

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_PLOTJUGGLER_SINK_HPP
