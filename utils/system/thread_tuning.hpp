/**
 * @file utils/system/thread_tuning.hpp
 * @brief ⭐⭐ **线程调度微调**（不需要 root 就能生效的那些）
 *
 * ## 为什么不做 `SCHED_FIFO`
 * 实测本机 `RLIMIT_RTPRIO = 0` → ⚠️ **普通用户设 `SCHED_FIFO` 会失败**（需要 `CAP_SYS_NICE`）。
 * ⇒ 本文件只做**零权限可用**的两件事：
 *
 * | 措施 | 需要 root | 效果 |
 * |---|---|---|
 * | ⭐ `SCHED_IDLE`（worker 用） | ❌ | ⭐ **只在 CPU 空闲时才跑 → 不抢自瞄线程** |
 * | ⭐ CPU 亲和性（P-core/E-core） | ❌ | 可把线程分开，避免同核争抢 |
 *
 * ## 实测的 CPU 拓扑（i7-14700HX，28 逻辑核）
 * ```
 *   P-core（5300~5500 MHz）: cpu0  ~ cpu15   （8 物理核 × 超线程）
 *   E-core（3900 MHz）:      cpu16 ~ cpu27   （12 物理核，无超线程）
 * ```
 *
 * ## 用法
 * ```cpp
 * // sink worker 线程开头：
 * tools::set_idle_policy();      // ⭐ 让出 CPU（失败也只是不生效，不报错终止）
 * ```
 */
#ifndef HZMIR_UTILS_SYSTEM_THREAD_TUNING_HPP
#define HZMIR_UTILS_SYSTEM_THREAD_TUNING_HPP

namespace tools
{

/// @brief ⭐ 把**当前线程**设为 `SCHED_IDLE`（最低优先级，只在 CPU 空闲时运行）
/// @return 是否成功（失败通常是因为内核不支持；**不抛异常、不终止**）
/// @note 不需要 root。适合 **sink worker** 这类"有就做、没有也不影响自瞄"的后台线程。
bool set_idle_policy();

/// @brief 探测本机是否允许设实时优先级（`RLIMIT_RTPRIO > 0` 或有 `CAP_SYS_NICE`）
/// @note 仅用于启动时提示；**为 false 时不要尝试 `SCHED_FIFO`**（会失败）
bool realtime_priority_available();

/// @brief ⭐ 把**当前线程**绑到指定 CPU 列表（如 P-core `"0-15"`）
/// @return 是否成功
bool pin_current_thread(const char * cpu_list);

/// @brief 启动时打印一次「线程调优可用性」（供使用者判断要不要上 `setcap`）
void report_thread_tuning();

}  // namespace tools

#endif  // HZMIR_UTILS_SYSTEM_THREAD_TUNING_HPP
