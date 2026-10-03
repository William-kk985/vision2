#ifndef IO__SOCKETCAN_HPP
#define IO__SOCKETCAN_HPP

#include <linux/can.h>
#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <thread>

#include "utils/log/logger.hpp"

using namespace std::chrono_literals;

constexpr int MAX_EVENTS = 10;

namespace io
{

/// @brief ⭐⭐ W109：**连续失败第 `n` 次时，该不该用 `warn`（否则 `debug`）**
///   ⚠️ 起因：守护线程**每 100ms 重试**，原来每次失败都 `warn`
///     ⇒ **每秒 10 条同样的警告**，实测把终端刷满。
///   ⭐ 判据：**第 1 次 + 每 50 次（约 5 秒）**报 `warn`，其余 `debug`
///     ⇒ 不刷屏，但仍能看出"一直在重试"。
///   ⚠️ **只影响日志级别，不影响重试行为**（还是 100ms 一次，连上就恢复）。
///   ⭐ 抽成独立函数是为了**可单测**（见 `test/function/test_can_throttle.cpp`）。
inline bool can_fail_should_warn(int n) { return n == 1 || (n > 0 && n % 50 == 0); }

class SocketCAN
{
public:
  SocketCAN(const std::string & interface, std::function<void(const can_frame & frame)> rx_handler)
  : interface_(interface),
    socket_fd_(-1),
    epoll_fd_(-1),
    rx_handler_(rx_handler),
    quit_(false),
    ok_(false),
    open_fail_count_(0)
  {
    try_open();

    // 守护线程
    daemon_thread_ = std::thread{[this] {
      while (!quit_) {
        std::this_thread::sleep_for(100ms);

        if (ok_) continue;

        if (read_thread_.joinable()) read_thread_.join();

        close();
        try_open();
      }
    }};
  }

  ~SocketCAN()
  {
    quit_ = true;
    if (daemon_thread_.joinable()) daemon_thread_.join();
    if (read_thread_.joinable()) read_thread_.join();
    close();
    tools::logger()->info("SocketCAN destructed.");
  }

  void write(can_frame * frame) const
  {
    if (::write(socket_fd_, frame, sizeof(can_frame)) == -1)
      throw std::runtime_error("Unable to write!");
  }

private:
  std::string interface_;
  int socket_fd_;
  int epoll_fd_;
  bool quit_;
  bool ok_;
  // ⭐⭐ W109：**连续失败次数** —— 只用于**日志降噪**（守护线程每 100ms 重试，
  //   原来每次都 `warn` ⇒ 每秒 10 条刷屏）。⚠️ **不影响重试行为**。
  int open_fail_count_;
  std::thread read_thread_;
  std::thread daemon_thread_;
  can_frame frame_;
  epoll_event events_[MAX_EVENTS];
  std::function<void(const can_frame & frame)> rx_handler_;

  void open()
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) throw std::runtime_error("Error opening socket!");

    ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
      throw std::runtime_error("Error getting interface index!");

    sockaddr_can addr;
    std::memset(&addr, 0, sizeof(sockaddr_can));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, (sockaddr *)&addr, sizeof(sockaddr_can)) < 0) {
      ::close(socket_fd_);
      throw std::runtime_error("Error binding socket to interface!");
    }

    epoll_event ev;
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ == -1) throw std::runtime_error("Error creating epoll file descriptor!");

    ev.events = EPOLLIN;
    ev.data.fd = socket_fd_;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, ev.data.fd, &ev))
      throw std::runtime_error("Error adding socket to epoll file descriptor!");

    // 接收线程
    read_thread_ = std::thread([this]() {
      ok_ = true;
      while (!quit_) {
        std::this_thread::sleep_for(10us);

        try {
          read();
        } catch (const std::exception & e) {
          tools::logger()->warn("SocketCAN::read() failed: {}", e.what());
          ok_ = false;
          break;
        }
      }
    });

    tools::logger()->info("SocketCAN opened.");
  }

  void try_open()
  {
    try {
      open();
      // ⭐⭐ W109：**重连成功要报一声**（之前只有失败有声音 ⇒ 恢复了也不知道）
      if (open_fail_count_ > 0) {
        tools::logger()->info(
          "SocketCAN::open() 恢复（之前失败 {} 次，约 {:.1f} 秒）—— IMU 数据恢复",
          open_fail_count_, open_fail_count_ * 0.1);
        open_fail_count_ = 0;
      }
    } catch (const std::exception & e) {
      // ⭐⭐⭐ W109：**降噪** —— ⚠️ 这里原来【每次失败都 warn】，
      //   而守护线程**每 100ms 重试一次** ⇒ **每秒 10 条同样的警告**，
      //   实测把终端刷满（真机上没接 CAN 时必然发生）。
      //   ⭐ 同济原版就是"每次 warn"（`upstream/.../socketcan.hpp` 逐字相同），
      //     但那是**日志噪音**、不是算法行为 ⇒ 本项目按"日志专业化"处理。
      //   ⭐ 做法：**只在第 1 次 + 每 50 次（约 5 秒）报一次 `warn`**，
      //     其余降为 `debug` ⇒ **不刷屏，但仍能看出"一直在重试"**。
      //   ⚠️ **重试行为一个字没改**（还是 100ms 一次，连上就恢复）。
      ++open_fail_count_;
      if (can_fail_should_warn(open_fail_count_) && open_fail_count_ == 1) {
        tools::logger()->warn(
          "SocketCAN::open() failed: {} —— 接口 '{}' 不可用（没接 CAN / 没 ip link set up）。"
          "⭐ 之后每 5 秒报一次，**不影响自瞄**，但 **IMU/EKF 数据不可信**。"
          "（想彻底避开 CAN ⇒ 用 `--video=<录像>` 走 ReplayCBoard）",
          e.what(), interface_);
      } else if (can_fail_should_warn(open_fail_count_)) {
        tools::logger()->warn(
          "SocketCAN 仍未连上（已失败 {} 次，约 {:.0f} 秒）：{}", open_fail_count_,
          open_fail_count_ * 0.1, e.what());
      } else {
        tools::logger()->debug("SocketCAN::open() retry #{}: {}", open_fail_count_, e.what());
      }
    }
  }

  void read()
  {
    int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, 2);
    if (num_events == -1) throw std::runtime_error("Error wating for events!");

    for (int i = 0; i < num_events; i++) {
      ssize_t num_bytes = recv(socket_fd_, &frame_, sizeof(can_frame), MSG_DONTWAIT);
      if (num_bytes == -1) throw std::runtime_error("Error reading from SocketCAN!");

      rx_handler_(frame_);
    }
  }

  void close()
  {
    if (socket_fd_ == -1) return;
    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, socket_fd_, NULL);
    ::close(epoll_fd_);
    ::close(socket_fd_);
  }
};

}  // namespace io

#endif  // IO__SOCKETCAN_HPP