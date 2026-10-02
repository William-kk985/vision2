#include "utils/system/thread_tuning.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>

#include <cstring>

#include "utils/log/logger.hpp"

namespace tools
{

bool set_idle_policy()
{
  ::sched_param p{};
  p.sched_priority = 0;   // SCHED_IDLE 的优先级必须为 0
  // ⭐ SCHED_IDLE：只在没有其它可运行任务时才被调度
  //   ⇒ sink worker 用它 → 自瞄线程永远不会因为它而等待
  if (::pthread_setschedparam(::pthread_self(), SCHED_IDLE, &p) != 0) return false;
  return true;
}

bool realtime_priority_available()
{
  ::rlimit rl{};
  if (::getrlimit(RLIMIT_RTPRIO, &rl) != 0) return false;
  return rl.rlim_cur > 0;
}

bool pin_current_thread(const char * cpu_list)
{
  cpu_set_t set;
  CPU_ZERO(&set);
  // 解析 "0-3,8" 这类列表
  const char * s = cpu_list;
  while (*s) {
    char * end = nullptr;
    const long a = std::strtol(s, &end, 10);
    if (end == s) break;              // 解析失败
    long b = a;
    if (*end == '-') {
      const char * s2 = end + 1;
      b = std::strtol(s2, &end, 10);
      if (end == s2) break;
    }
    for (long i = a; i <= b && i < CPU_SETSIZE; ++i) CPU_SET(i, &set);
    if (*end != ',') break;
    s = end + 1;
  }
  return ::pthread_setaffinity_np(::pthread_self(), sizeof(set), &set) == 0;
}


}  // namespace tools
