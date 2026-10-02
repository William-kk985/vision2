#include "plotjuggler_sink.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sstream>

#include "utils/log/logger.hpp"
#include "utils/system/thread_tuning.hpp"   // ⭐ W82

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
  // ⭐⭐ W80：起 worker（JSON 拼装 + sendto 都在它上面）
  //   ⚠️⚠️ 这一行曾在 W80 被漏掉（Python 替换用了 4 空格缩进、文件是 2 空格 → 静默失败）：
  //      后果是 `on_frame` 只往队列推、**没人消费** → PlotJuggler 一条数据都收不到，
  //      而 benchmark 只测入队（104 ns）看不出问题。**加下面的启动日志便于自查。**
  th_ = std::thread([this] { worker(); });
  tools::logger()->info("[PlotJugglerSink] -> {}:{}（JSON 拼装在 worker 线程，不占自瞄）",
                        host, port);
}

PlotJugglerSink::~PlotJugglerSink()
{
  // ⭐ W80：先停 worker（否则它可能还在用 dest_/fd_）
  {
    std::lock_guard lk(mtx_);
    enabled_ = false;
    quit_.store(true);
  }
  cv_.notify_all();
  if (th_.joinable()) th_.join();

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

  // ⭐⭐⭐ W80：自瞄线程**只做**「取时间戳 + 400 B POD 拷贝 + 入队」
  //   原来这里拼 ~18 个字段的 `std::ostringstream` JSON（~1 µs）→ 已搬到 worker。
  //   ⚠️ `timestamp` 必须在这里取（worker 取会滞后到"发送时刻"而非"帧时刻"）。
  Queued item;
  item.ts = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
  item.d = d;

  {
    std::lock_guard lk(mtx_);
    if (!enabled_) return;
    if (q_.size() >= kMaxQueue) {
      q_.pop_front();                       // ⭐ 丢了最旧的（实时曲线无所谓）
      dropped_.fetch_add(1, std::memory_order_relaxed);
    }
    q_.push_back(item);
  }
  cv_.notify_one();
}

void PlotJugglerSink::worker()
{
  // ⭐⭐ W82：worker 用 SCHED_IDLE —— 只在 CPU 空闲时才跑，
  //   保证【永远不抢自瞄线程】（不需要 root；失败也只是不生效）
  tools::set_idle_policy();
  while (true) {
    Queued item;
    {
      std::unique_lock lk(mtx_);
      cv_.wait(lk, [this] { return quit_.load() || !q_.empty(); });
      if (q_.empty()) {
        if (quit_.load()) break;
        continue;
      }
      item = q_.front();
      q_.pop_front();
    }

    // ⭐⭐ 以下全在 worker 线程（不再占自瞄线程）
    const auto & d = item.d;
    std::ostringstream o;
    o << "{\"timestamp\":" << item.ts
      << ",\"frame_id\":" << d.frame_id << ",\"t_frame_us\":" << d.t_frame_us
      << ",\"t_perceive_us\":" << d.t_perceive_us << ",\"t_decide_us\":" << d.t_decide_us
      << ",\"det_armor_count\":" << d.detector.armor_count
      << ",\"det_t_infer_us\":" << d.detector.t_infer_us
      << ",\"trk_state\":" << d.tracker.state
      << ",\"tgt_w\":" << d.target.w << ",\"tgt_nis\":" << d.target.nis
      << ",\"tgt_x\":" << d.target.xyz_world[0] << ",\"tgt_y\":" << d.target.xyz_world[1]
      << ",\"tgt_z\":" << d.target.xyz_world[2]
      << ",\"pln_t_fly\":" << d.planner.t_fly << ",\"pln_overlap\":" << d.planner.overlap_ratio
      << ",\"pln_iters\":" << d.planner.solver_iters
      << ",\"sht_should_fire\":" << (d.shooter.should_fire ? 1 : 0)
      << ",\"ctl_yaw\":" << d.controller.cmd_yaw << ",\"ctl_pitch\":" << d.controller.cmd_pitch
      << "}";
    send(o.str());
  }
}

}  // namespace tools
