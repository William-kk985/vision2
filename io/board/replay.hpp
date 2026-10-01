/**
 * @file io/board/replay.hpp
 * @brief ⭐ 录像回放的「下位机」：回放云台四元数 + 固定档位 + 记录下发
 *
 * 没有下位机时，用这个驱动 plan 线程：
 *   · `q(t)`  —— 从同名 `.txt` 逐行回放 `<t> <w> <x> <y> <z>`（无文件则单位四元数）
 *   · `mode()`—— 固定成一个档位（默认 AUTO_AIM），让主循环真的跑起来
 *   · `send()`—— 不上发，只**记录**（用于事后分析下发序列）
 */
#ifndef HZMIR_IO_BOARD_REPLAY_HPP
#define HZMIR_IO_BOARD_REPLAY_HPP

#include <fstream>
#include <string>
#include <vector>

#include "io/board/board.hpp"

namespace io
{

class ReplayBoard : public IBoard
{
public:
  /// @param pose_path 位姿文件（空 = 单位四元数）；与视频同名 `.txt`
  explicit ReplayBoard(
    const std::string & pose_path = "", GimbalMode mode = GimbalMode::AUTO_AIM,
    float bullet_speed = 22.0f, uint16_t bullet_count = 100);

  GimbalMode mode() const override { return mode_; }
  GimbalState state() const override;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t) override;

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc) override;

  // ── 事后分析用 ──
  struct Sent
  {
    bool control, fire;
    float yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc;
  };
  const std::vector<Sent> & sent_log() const { return sent_log_; }
  void clear_log() { sent_log_.clear(); }
  bool pose_ok() const { return pose_.is_open(); }

private:
  GimbalMode mode_;
  float bullet_speed_;
  uint16_t bullet_count_;

  std::ifstream pose_;
  std::vector<Sent> sent_log_;
  uint16_t sent_count_ = 0;
};

}  // namespace io

#endif  // HZMIR_IO_BOARD_REPLAY_HPP
