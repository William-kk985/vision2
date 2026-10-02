#include "io/board/replay.hpp"

#include <cmath>

#include "utils/log/logger.hpp"

namespace io
{

ReplayBoard::ReplayBoard(
  const std::string & pose_path, GimbalMode mode, float bullet_speed, uint16_t bullet_count)
: mode_(mode), bullet_speed_(bullet_speed), bullet_count_(bullet_count)
{
  if (!pose_path.empty()) {
    pose_.open(pose_path);
    if (pose_.is_open())
      tools::logger()->debug("[ReplayBoard] 位姿文件: {}", pose_path);
    else
      tools::logger()->warn("[ReplayBoard] 打不开位姿文件 {} → 用单位四元数", pose_path);
  }
  tools::logger()->info(
    "[ReplayBoard] 固定档位 mode={} bullet_speed={:.1f}", static_cast<int>(mode_), bullet_speed_);
}

GimbalState ReplayBoard::state() const
{
  GimbalState s{};
  s.yaw = 0;
  s.yaw_vel = 0;
  s.pitch = 0;
  s.pitch_vel = 0;
  s.bullet_speed = bullet_speed_;
  s.bullet_count = bullet_count_;
  return s;
}

Eigen::Quaterniond ReplayBoard::q(std::chrono::steady_clock::time_point /*t*/)
{
  if (!pose_.is_open()) return Eigen::Quaterniond::Identity();

  double ts, w, x, y, z;
  if (pose_ >> ts >> w >> x >> y >> z) {
    Eigen::Quaterniond q(w, x, y, z);
    q.normalize();
    return q;
  }
  return Eigen::Quaterniond::Identity();   // 文件读完 → 保持单位
}

void ReplayBoard::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  sent_log_.push_back({control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc});
  ++sent_count_;
}

}  // namespace io
