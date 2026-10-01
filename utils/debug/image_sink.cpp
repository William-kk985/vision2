#include "utils/debug/image_sink.hpp"

#include <opencv2/imgcodecs.hpp>

#include <sys/stat.h>

#include "utils/log/logger.hpp"

namespace tools
{

ImageSink::ImageSink(std::string out_dir, int every_n, size_t max_files)
: out_dir_(std::move(out_dir)), every_n_(every_n < 1 ? 1 : every_n), max_files_(max_files)
{
  ::mkdir(out_dir_.c_str(), 0755);   // 已存在则忽略错误
  th_ = std::thread([this] { worker(); });
  tools::logger()->info(
    "[ImageSink] -> {}  每 {} 张存 1 张，上限 {} 张", out_dir_, every_n_, max_files_);
}

ImageSink::~ImageSink() { close(); }

void ImageSink::on_image(std::string_view tag, const cv::Mat & img, int64_t t_us)
{
  (void)t_us;
  if (img.empty()) return;

  if (seen_++ % static_cast<size_t>(every_n_) != 0) return;

  // ⭐ W19 修复：用 `accepted_`（只增不减）判断上限，而不是会被 worker 消费的 `q_.size()`
  if (accepted_.load() >= max_files_) {
    dropped_.fetch_add(1);
    if (!capped_warned_) {
      capped_warned_ = true;
      tools::logger()->warn("[ImageSink] 已达上限 {} 张 → 停止存图", max_files_);
    }
    return;
  }
  accepted_.fetch_add(1);

  const std::string path = out_dir_ + "/" + cv::format("%06u_%s.png", frame_id_,
                                                       std::string(tag).c_str());
  {
    std::lock_guard lk(mtx_);
    q_.emplace_back(path, img.clone());   // ⭐ 热路径只 clone + 入队，不落盘
  }
  cv_.notify_one();
}

void ImageSink::worker()
{
  while (true) {
    std::pair<std::string, cv::Mat> item;
    {
      std::unique_lock lk(mtx_);
      cv_.wait(lk, [this] { return quit_ || !q_.empty(); });
      if (q_.empty()) {
        if (quit_) break;
        continue;
      }
      item = std::move(q_.front());
      q_.pop_front();
    }
    if (!item.second.empty() && cv::imwrite(item.first, item.second)) saved_.fetch_add(1);
  }
}

void ImageSink::close()
{
  {
    std::lock_guard lk(mtx_);
    quit_ = true;
  }
  cv_.notify_all();
  if (th_.joinable()) th_.join();
  if (saved_.load() > 0 || dropped_.load() > 0)
    tools::logger()->info(
      "[ImageSink] 结束：写入 {} 张，丢弃 {}", saved_.load(), dropped_.load());
}

}  // namespace tools
