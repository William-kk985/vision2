/**
 * @file utils/debug/expense.hpp
 * @brief ⭐ 耗时面板（学哈工程的 `DebugExpense.msg`）
 *
 * **L0 常驻，零宏**：`steady_clock::now()` ~20 ns，一段 ~40 ns，
 * 5 段合计 ~200 ns/帧 → 相对 10 ms 帧周期是 **0.002%**，可以永远开着。
 */
#ifndef HZMIR_UTILS_DEBUG_EXPENSE_HPP
#define HZMIR_UTILS_DEBUG_EXPENSE_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace tools
{

class Expense
{
public:
  using clock = std::chrono::steady_clock;

  /// @brief 开始一段计时
  void begin(const char * tag) { t0_[tag] = clock::now(); }

  /// @brief 结束一段计时，累加到该 tag（返回本次耗时 us）
  int64_t end(const char * tag)
  {
    auto it = t0_.find(tag);
    if (it == t0_.end()) return 0;
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - it->second).count();
    us_[tag] += us;
    ++count_[tag];
    t0_.erase(it);
    return us;
  }

  /// @brief 每帧末尾调用：把本帧累计搬进「上一帧」，然后清空本帧
  void next_frame()
  {
    last_ = us_;
    last_count_ = count_;      // ⭐ B3 修复：原来 last_count_ **从未赋值** → count() 恒返回 0
    us_.clear();
    count_.clear();
  }

  /// ⭐ B3 修复：`us()` 原来读的是 `last_`（**上一帧**）→ 当帧耗时恒读到 0。
  ///    现在 `us()` 读**本帧**；要看上一帧用 `last_us()`。
  int64_t us(const char * tag) const
  {
    auto it = us_.find(tag);
    return it == us_.end() ? 0 : it->second;
  }
  int64_t last_us(const char * tag) const
  {
    auto it = last_.find(tag);
    return it == last_.end() ? 0 : it->second;
  }
  int64_t count(const char * tag) const
  {
    auto it = count_.find(tag);
    return it == count_.end() ? 0 : it->second;
  }

  /// @brief 一行可读摘要（用于 logger）—— 用**本帧**数据
  std::string summary() const;
  /// @brief 上一帧的摘要
  std::string last_summary() const;

private:
  std::unordered_map<std::string, clock::time_point> t0_;
  std::unordered_map<std::string, int64_t> us_, last_, count_, last_count_;
};

/// @brief RAII 作用域计时（比 begin/end 更难写错）
class ScopedExpense
{
public:
  ScopedExpense(Expense & e, const char * tag) : e_(e), tag_(tag) { e_.begin(tag_); }
  ~ScopedExpense() { e_.end(tag_); }
  ScopedExpense(const ScopedExpense &) = delete;
  ScopedExpense & operator=(const ScopedExpense &) = delete;

private:
  Expense & e_;
  const char * tag_;
};

#define HZMIR_EXPENSE(e, tag) ::tools::ScopedExpense _hzmir_exp_##__LINE__(e, tag)

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_EXPENSE_HPP
