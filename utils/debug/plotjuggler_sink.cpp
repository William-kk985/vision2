#include "plotjuggler_sink.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sstream>

#include "utils/log/logger.hpp"

namespace tools
{

PlotJugglerSink::PlotJugglerSink(std::string host, uint16_t port, bool enabled)
: enabled_(enabled), t0_(std::chrono::steady_clock::now())
{
  if (!enabled_) return;
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  auto * d = new ::sockaddr_in{};
  d->sin_family = AF_INET;
  d->sin_port = ::htons(port);
  d->sin_addr.s_addr = ::inet_addr(host.c_str());
  dest_ = d;
  tools::logger()->info("[PlotJugglerSink] -> {}:{}", host, port);
}

PlotJugglerSink::~PlotJugglerSink()
{
  if (fd_ >= 0) ::close(fd_);
  delete dest_;
  dest_ = nullptr;
}

void PlotJugglerSink::send(const std::string & json)
{
  if (!enabled_ || fd_ < 0 || !dest_) return;
  ::sendto(fd_, json.c_str(), json.length(), 0, reinterpret_cast<::sockaddr *>(dest_), sizeof(*dest_));
}

void PlotJugglerSink::on_frame(const auto_aim::FrameDebug & d)
{
  if (!enabled_) return;
  const double ts = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();

  std::ostringstream o;
  o << "{\"timestamp\":" << ts   // ⭐ 必须有，否则落不了时间轴
    << ",\"frame_id\":" << d.frame_id << ",\"t_frame_us\":" << d.t_frame_us
    << ",\"t_perceive_us\":" << d.t_perceive_us << ",\"t_decide_us\":" << d.t_decide_us
    << ",\"det_armor_count\":" << d.detector.armor_count
    << ",\"det_t_infer_us\":" << d.detector.t_infer_us
    << ",\"trk_state\":" << d.tracker.state
    << ",\"tgt_w\":" << d.target.w << ",\"tgt_nis\":" << d.target.nis
    << ",\"tgt_x\":" << d.target.xyz_world[0] << ",\"tgt_y\":" << d.target.xyz_world[1]
    << ",\"tgt_z\":" << d.target.xyz_world[2]
    << ",\"pln_t_fly\":" << d.planner.t_fly << ",\"pln_overlap\":" << d.planner.overlap_ratio
    << ",\"pln_kill_time\":" << d.planner.kill_time << ",\"pln_iters\":" << d.planner.solver_iters
    << ",\"sht_should_fire\":" << (d.shooter.should_fire ? 1 : 0)
    << ",\"ctl_yaw\":" << d.controller.cmd_yaw << ",\"ctl_pitch\":" << d.controller.cmd_pitch
    << "}";
  send(o.str());
}

}  // namespace tools
