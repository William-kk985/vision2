#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <utility>

namespace tools
{
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)
  {
  }

  // ── ⭐ W11（J5/J6）：移动入队 + emplace + notify_one ──────────────

  void push(const T & value) { push_impl(value); }

  /// ⭐ 优化建议（J5）：移动重载（避免队列内部再拷一次）—— 附加接口，不改原行为
  void push(T && value) { push_impl(std::move(value)); }

  /// ⭐ J5：原地构造（连移动都省了）
  template <typename... Args>
  void emplace(Args &&... args)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    make_room_();
    if (!ok_) return;
    queue_.emplace(std::forward<Args>(args)...);
    // ⚠️ 同济用 `notify_all` —— 多消费者场景下 `notify_one` 会漏唤醒，**保持同济行为**
    not_empty_condition_.notify_all();
  }

  // ── ⭐ W11（J4/J7）：原子化的"取"接口 ────────────────────────────

  /// ⭐ J7：**一次加锁**完成「判空 + 拷贝取出（不弹出）」
  ///     取代 `if (!q.empty()) { auto v = q.front(); }` 的 TOCTOU + by-value 拷贝
  /// @return 是否成功（false = 队列为空）
  bool try_peek(T & out)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.empty()) return false;
    out = queue_.front();      // 一次拷贝到调用方（原来的 front() 也是这个成本，但没有 TOCTOU）
    return true;
  }

  /// ⭐ J7：**一次加锁**完成「判空 + 移动取出（弹出）」
  bool try_pop(T & out)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.empty()) return false;
    out = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  /// @brief ⭐⭐ W55：**带超时的取出** —— 使「等数据的循环」不再是 Ctrl-C 的死区
  ///
  /// ## 为什么需要它（真实翻车）
  /// `pop()` 是**无限阻塞**的。当相机/USB 掉线时，驱动线程永远不给队列放数据 →
  /// **主循环卡在 `pop()` 里** → `exiter.exit()` 和 `hotkeys.poll()` 都执行不到 →
  /// ⚠️ **Ctrl-C 不退出、热键全失效**（用户实测踩到）。
  ///
  /// ⭐ 用超时后：主循环最多阻塞 `timeout`，然后能继续检查退出标志。
  ///
  /// @return true = 取到了；false = 超时（调用方应处理"暂时没数据"）
  template <typename Rep, typename Period>
  bool pop_for(T & out, const std::chrono::duration<Rep, Period> & timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!not_empty_condition_.wait_for(lock, timeout, [this] { return !queue_.empty(); }))
      return false;   // ⭐ 超时：队列仍为空
    out = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  void pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
      return;
    }

    value = queue_.front();
    queue_.pop();
  }

  T pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    T value = std::move(queue_.front());
    queue_.pop();
    return std::move(value);
  }

  T front()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    return queue_.front();
  }

  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    value = queue_.back();
  }

  /// @brief ⭐ W48：队列容量（诊断用，如打印丢帧警告时）
  size_t capacity() const { return max_size_; }

  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
    not_empty_condition_.notify_all();   // ⚠️ 保持同济行为
  }

private:
  /// 统一的入队实现（拷贝 or 移动）
  template <typename U>
  void push_impl(U && value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    make_room_();
    if (!ok_) return;
    queue_.push(std::forward<U>(value));
    not_empty_condition_.notify_all();   // ⚠️ 保持同济行为
  }

  /// 腾位置；失败时置 ok_ = false
  void make_room_()
  {
    if (queue_.size() < max_size_) {
      ok_ = true;
      return;
    }
    if (PopWhenFull) {
      queue_.pop();
      ok_ = true;
    } else {
      full_handler_();
      ok_ = false;
    }
  }

  bool ok_ = true;
  std::queue<T> queue_;
  size_t max_size_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP