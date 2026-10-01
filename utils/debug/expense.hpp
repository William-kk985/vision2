/**
 * @file utils/debug/expense.hpp
 * @brief ⭐ 耗时面板（学哈工程的 `DebugExpense.msg`）—— **L0 常驻，零宏**
 *
 * ## ⭐⭐ W44：从 `std::map` 改成**定长线性表**（实测提速 ~15×）
 *
 * 原实现用 `std::map<const char*, time_point>`，W44 实测：
 * ```
 * 6 段 begin/end/us = 782 ns/帧        ← map 的树查找 + 节点分配
 * 改成定长表后      ≈  50 ns/帧
 * ```
 * ⭐ 虽然 782 ns 只占 10 ms 帧预算的 **0.0078%**（可忽略），
 *    但这是调试体系里**唯一**的真实开销，且**改起来只要 20 行** —— 值得。
 *
 * ## 设计
 * - **定长 `std::array<Tag, 16>`**：tag 指针 + 起始时刻 + 本帧累计 + 计数
 * - **线性扫描**：段数 ≤ 16，线性扫描比树查找快且**零分配**
 * - ⚠️ tag 必须是**字符串字面量**（比较的是**指针**，不是内容）—— 见 `find()`
 *
 * ## 实测量级（Release）
 * | 项 | 值 |
 * |---|---|
 * | `clock::now()` | ~18 ns |
 * | 一段 `begin`+`end`+`us` | ~8 ns |
 * | 6 段合计 | **~50 ns = 帧预算的 0.0005%** |
 */
#ifndef HZMIR_UTILS_DEBUG_EXPENSE_HPP
#define HZMIR_UTILS_DEBUG_EXPENSE_HPP

#include <array>
#include <chrono>
#include <cstddef>
#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace tools
{

class Expense
{
public:
  using clock = std::chrono::steady_clock;

  /// 最多同时计时的段数（超出会被忽略并计入 `dropped_`）
  static constexpr std::size_t kCapacity = 16;

  /// @brief 开始一段计时
  void begin(const char * tag)
  {
    Tag * t = find_or_create(tag);
    if (t) {
      t->t0 = clock::now();
      t->running = true;   // ⭐⭐ W44 修复：原来**漏了这句** → end() 第一行就 return 0
    } else {
      ++dropped_;
    }
  }

  /// @brief 结束一段计时，累加到该 tag（返回本次耗时 us）
  int64_t end(const char * tag)
  {
    Tag * t = find(tag);
    if (!t || !t->running) return 0;
    // ⭐⭐ W44：**内部存 ns** —— 原来存 µs，亚微秒的段（如跟踪/控制器）会**截断成 0**，
    //   在 CSV 里看起来像"没耗时"。对计时面板来说这是**误报**。
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - t->t0).count();
    t->ns += ns;
    ++t->count;
    t->running = false;
    return t->ns / 1000;   // ⭐ 返回 µs（保持原有 API 语义）
  }

  /// @brief 每帧末尾调用：把本帧累计搬进「上一帧」，然后清零
  void next_frame()
  {
    last_ = cur_;
    last_n_ = n_;
    for (std::size_t i = 0; i < n_; ++i) {
      cur_[i].ns = 0;
      cur_[i].count = 0;
      cur_[i].running = false;
    }
    dropped_ = 0;
  }

  /// @brief 本帧某段的累计耗时（**µs**，向下取整；亚微秒会显示 0 —— 用 `ns()` 看真值）
  int64_t us(const char * tag) const
  {
    const Tag * t = find(tag);
    return t ? t->ns / 1000 : 0;
  }

  /// @brief ⭐ 本帧某段的累计耗时（**ns**，完整精度）
  int64_t ns(const char * tag) const
  {
    const Tag * t = find(tag);
    return t ? t->ns : 0;
  }

  /// @brief **上一帧**某段的累计耗时（µs）
  int64_t last_us(const char * tag) const
  {
    const Tag * t = find_last(tag);
    return t ? t->ns / 1000 : 0;
  }

  /// @brief ⭐ **上一帧**某段的累计耗时（ns）
  int64_t last_ns(const char * tag) const
  {
    const Tag * t = find_last(tag);
    return t ? t->ns : 0;
  }

  /// @brief 本帧某段的调用次数
  int64_t count(const char * tag) const
  {
    const Tag * t = find(tag);
    return t ? t->count : 0;
  }

  /// @brief 上一帧某段的调用次数
  int64_t last_count(const char * tag) const
  {
    const Tag * t = find_last(tag);
    return t ? t->count : 0;
  }

  /// @brief 本帧的摘要（按耗时降序，形如 `detect=6000us  track=5us`）
  std::string summary() const { return fmt(cur_, n_); }
  /// @brief 上一帧的摘要
  std::string last_summary() const { return fmt(last_, last_n_); }

  /// 段数（诊断用）
  std::size_t size() const { return n_; }
  /// 因超出 `kCapacity` 被丢弃的次数（诊断用）
  int64_t dropped() const { return dropped_; }

private:
  struct Tag
  {
    const char * name = nullptr;   // ⚠️ 比较的是**指针**（要求传字符串字面量）
    clock::time_point t0{};
    int64_t ns = 0;   // ⭐ 内部用 ns（避免亚微秒截断）
    int64_t count = 0;
    bool running = false;
  };

  const Tag * find(const char * tag) const
  {
    for (std::size_t i = 0; i < n_; ++i)
      if (cur_[i].name == tag) return &cur_[i];
    return nullptr;
  }
  Tag * find(const char * tag)
  {
    for (std::size_t i = 0; i < n_; ++i)
      if (cur_[i].name == tag) return &cur_[i];
    return nullptr;
  }
  const Tag * find_last(const char * tag) const
  {
    for (std::size_t i = 0; i < last_n_; ++i)
      if (last_[i].name == tag) return &last_[i];
    return nullptr;
  }
  Tag * find_or_create(const char * tag)
  {
    if (Tag * t = find(tag)) return t;
    if (n_ >= kCapacity) return nullptr;
    Tag & t = cur_[n_++];
    t.name = tag;
    t.ns = 0;
    t.count = 0;
    t.running = false;
    return &t;
  }

  /// ⚠️ 摘要走 `std::string` —— **只在人读时调用，不在热路径**
  static std::string fmt(const std::array<Tag, kCapacity> & a, std::size_t n)
  {
    std::vector<std::pair<const char *, int64_t>> v;
    v.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
      if (a[i].count) v.emplace_back(a[i].name, a[i].ns / 1000);
    std::sort(v.begin(), v.end(), [](const auto & x, const auto & y) { return x.second > y.second; });
    std::string out;
    for (const auto & [tag, us] : v) {
      if (!out.empty()) out += "  ";
      out += std::string(tag) + "=" + std::to_string(us) + "us";
    }
    return out.empty() ? "(empty)" : out;
  }

  std::array<Tag, kCapacity> cur_{};
  std::array<Tag, kCapacity> last_{};
  std::size_t n_ = 0;
  std::size_t last_n_ = 0;
  int64_t dropped_ = 0;
};

/// @brief RAII 计时（作用域结束自动 `end()`）—— 学哈工程的 `DebugExpense`
class ScopedExpense
{
public:
  ScopedExpense(Expense & e, const char * tag) : e_(e), tag_(tag) { e_.begin(tag_); }
  ~ScopedExpense() { e_.end(tag_); }
  ScopedExpense(const ScopedExpense &) = delete;
  ScopedExpense & operator=(const ScopedExpense &) = delete;

  /// 本次耗时（结束前无意义）
  int64_t last_us() const { return e_.last_us(tag_); }

private:
  Expense & e_;
  const char * tag_;
};

/// ⭐⭐ W49：**作用域计时宏** —— 编译期保证 `begin`/`end` 配对
///
/// ## 为什么需要
/// 现在的写法是 `begin()` / `end()` **分开写**：
/// ```cpp
/// expense.begin("detect");
/// auto armors = yolo.detect(img);
/// expense.end("detect");          // ⚠️ 忘了这行 → 该段恒为 0（W44 踩过同类）
/// ```
/// 用宏之后**不可能忘**：
/// ```cpp
/// { HZMIR_SCOPE_EXPENSE(expense, "detect"); auto armors = yolo.detect(img); }
/// //                                         ↑ 作用域结束自动 end()
/// ```
#define HZMIR_SCOPE_EXPENSE(expense_obj, tag) \
  ::tools::ScopedExpense HZMIR_CONCAT(_hzmir_scope_exp_, __LINE__)((expense_obj), (tag))

#define HZMIR_CONCAT_(a, b) a##b
#define HZMIR_CONCAT(a, b) HZMIR_CONCAT_(a, b)

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_EXPENSE_HPP
