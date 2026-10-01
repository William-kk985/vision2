/**
 * @file utils/debug/csv_sink.hpp
 * @brief ⭐ CSV sink —— **离线数据分析的主力**（Pandas / scripts/*.py）
 *
 * 输出两个文件：
 *   `<prefix>_frames.csv`  每帧一行（L0 + L1 全部字段）
 *   `<prefix>_series.csv`  `key,t_us,value`（L2 曲线）
 *
 * ⚠️ 落盘在**独立线程**（doc 09 §14.3 约束 1：热路径只填结构体，不做 IO）
 */
#ifndef HZMIR_UTILS_DEBUG_CSV_SINK_HPP
#define HZMIR_UTILS_DEBUG_CSV_SINK_HPP

#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "debug_sink.hpp"

namespace tools
{

class CsvSink : public IDebugSink
{
public:
  /// @param prefix 输出文件前缀（如 "hzmir" → hzmir_frames.csv / hzmir_series.csv）
  /// @param max_rows ⭐ 安全上限（默认 200 万帧 ≈ 数小时 @100Hz）。
  ///        超过后**停止入队并只警告一次** —— 防止上游空转把磁盘写爆（实测过一次 451 MB）。
  explicit CsvSink(std::string prefix = "hzmir", size_t max_rows = 2'000'000);
  ~CsvSink() override;

  const char * name() const override { return "csv"; }

  void on_frame(const auto_aim::FrameDebug & d) override;
  void on_series(std::string_view key, int64_t t_us, double v) override;

  /// @brief 冲刷并停止（析构会自动调）
  void close();

  const std::string & frames_path() const { return frames_path_; }
  const std::string & series_path() const { return series_path_; }

private:
  void worker();

  std::string frames_path_, series_path_;
  FILE * ff_ = nullptr, * sf_ = nullptr;

  std::thread th_;
  std::mutex mtx_;
  std::condition_variable cv_;
  bool quit_ = false;
  std::deque<std::string> q_;     // 待写文本（在 worker 里落盘）
  size_t max_rows_ = 2'000'000;
  size_t dropped_ = 0;
  size_t rows_ = 0;
  bool capped_warned_ = false;
  bool shutdown_log_ = true;
};

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_CSV_SINK_HPP
