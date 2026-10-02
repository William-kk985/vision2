#include "csv_sink.hpp"

#include <sstream>

#include "utils/log/logger.hpp"
#include "utils/system/paths.hpp"   // ⭐ W87
#include "utils/system/thread_tuning.hpp"   // ⭐ W82

namespace tools
{

namespace
{
const char * FRAME_HEADER =
  "frame_id,t_frame_us,t_cam_wait_us,t_perceive_us,t_decide_us,mode,game_state,"
  // detector
  "det_armor_count,det_best_conf,det_nms,det_t_infer_us,"
  // solver
  "sol_solved,sol_reproj_err,sol_yaw_offset,sol_t_solve_us,"
  // tracker
  "trk_state,trk_armor_count,trk_priority_mode,trk_filtered_out,trk_invincible_count,trk_focus_target_count,trk_t_track_us,"
  // target
  "tgt_id,tgt_x,tgt_y,tgt_z,tgt_yaw,tgt_w,tgt_r,tgt_l,tgt_h,tgt_nis,tgt_nis_thresh,"
  "tgt_invincible,tgt_t_update_us,"
  // planner
  "pln_t_fly,pln_overlap,pln_iters,pln_acc_max,"
  "pln_t_plan_us,"
  // shooter
  "sht_traj_err,sht_fire_thresh,sht_should_fire,sht_blocked_inv,sht_blocked_filter,"
  "sht_t_since_fire_us,"
  // controller
  "ctl_yaw,ctl_pitch,ctl_control,ctl_shoot,ctl_t_us,"
  // ⭐ W27：打符（原来漏了 —— BuffDebug 一直是死字段）
  "buff_rune_type,buff_fanblade_count,buff_spd,buff_solved,buff_t_us\n";

std::string row(const auto_aim::FrameDebug & d)
{
  std::ostringstream o;
  auto b = [](bool v) { return v ? 1 : 0; };
  o << d.frame_id << ',' << d.t_frame_us << ',' << d.t_cam_wait_us << ',' << d.t_perceive_us
    << ',' << d.t_decide_us << ','
    << int(d.mode) << ',' << int(d.game_state) << ','
    << d.detector.armor_count << ',' << d.detector.best_confidence << ',' << d.detector.nms_survivors
    << ',' << d.detector.t_infer_us << ','
    << d.solver.solved << ',' << d.solver.reprojection_error << ',' << d.solver.yaw_offset << ','
    << d.solver.t_solve_us << ','
    << d.tracker.state << ',' << d.tracker.armor_count << ',' << d.tracker.priority_mode << ','
    << d.tracker.filtered_out << ',' << d.tracker.invincible_count << ','
    << d.tracker.focus_target_count << ',' << d.tracker.t_track_us << ','
    << d.target.tracked_id << ',' << d.target.xyz_world[0] << ',' << d.target.xyz_world[1] << ','
    << d.target.xyz_world[2] << ',' << d.target.yaw << ',' << d.target.w << ',' << d.target.r << ','
    << d.target.l << ',' << d.target.h << ',' << d.target.nis << ',' << d.target.nis_thresh << ','
    << b(d.target.invincible) << ',' << d.target.t_update_us << ','
    // ⚠️ W72：删掉 t_fire / t_pred / dps / kill_time（**算法里没这些量**，留着只会是 0）
    << d.planner.t_fly << ',' << d.planner.overlap_ratio << ',' << d.planner.solver_iters << ','
    << d.planner.acc_max << ',' << d.planner.t_plan_us << ','
    << d.shooter.traj_err_at_fire << ',' << d.shooter.fire_thresh << ','
    << b(d.shooter.should_fire) << ',' << b(d.shooter.blocked_by_invincible) << ','
    << b(d.shooter.blocked_by_filter) << ',' << d.shooter.t_since_last_fire_us << ','
    << d.controller.cmd_yaw << ',' << d.controller.cmd_pitch << ','
    << b(d.controller.control) << ',' << b(d.controller.shoot) << ',' << d.controller.t_ctrl_us
    << ','
    // ⭐ W27：打符
    << d.buff.rune_type << ',' << d.buff.fanblade_count << ',' << d.buff.spd << ','
    << b(d.buff.solved) << ',' << d.buff.t_us << '\n';
  return o.str();
}
}  // namespace

CsvSink::CsvSink(std::string prefix, size_t max_rows) : max_rows_(max_rows)
{
  // ⭐ W87：CSV → `output/csv`（除非用户给的 prefix 里已含 `/`，那就尊重原路径）
  const std::string dir_prefix = tools::paths::csv_prefix(prefix);
  tools::paths::ensure_dir(tools::paths::csv());
  frames_path_ = dir_prefix + "_frames.csv";
  series_path_ = dir_prefix + "_series.csv";

  ff_ = std::fopen(frames_path_.c_str(), "w");
  sf_ = std::fopen(series_path_.c_str(), "w");
  if (!ff_ || !sf_) {
    tools::logger()->error("[CsvSink] 打不开输出文件: {} / {}", frames_path_, series_path_);
    return;
  }
  std::fputs(FRAME_HEADER, ff_);
  std::fputs("key,t_us,value\n", sf_);

  th_ = std::thread([this] { worker(); });
  tools::logger()->info("[CsvSink] -> {} , {}", frames_path_, series_path_);
}

CsvSink::~CsvSink() { close(); }

void CsvSink::on_frame(const auto_aim::FrameDebug & d)
{
  // ⭐⭐ W46：**自瞄线程只做一次 400 B POD 拷贝 + 入队**；`row()` 的格式化(~7 µs)推到 worker
  std::lock_guard lk(mtx_);
  // ⭐ 安全上限：超过就丢弃（并只警告一次），避免上游空转写爆磁盘
  if (rows_ + qf_.size() >= max_rows_) {
    ++dropped_;
    if (!capped_warned_) {
      capped_warned_ = true;
      tools::logger()->error(
        "[CsvSink] 已达上限 {} 帧（很可能是上游空转）→ 停止写入。"
        "请检查主循环是否在空帧时退出；本帧起丢弃。",
        max_rows_);
    }
    return;
  }
  if ((rows_ + qf_.size()) % 100000 == 0 && rows_ + qf_.size() > 0)
    tools::logger()->info("[CsvSink] 已写 {} 帧...", rows_ + qf_.size());
  qf_.push_back(d);          // ⭐ 400 B POD 拷贝
  cv_.notify_one();
}

void CsvSink::on_series(std::string_view key, int64_t t_us, double v)
{
  std::ostringstream o;
  o << key << ',' << t_us << ',' << v << '\n';   // ⭐⭐ W46：删掉旧的 "\nseries\n" 尾标记（现在分两个队列了，尾标记会让每行多一条 "series"）
  std::lock_guard lk(mtx_);
  qs_.push_back(std::move(o.str()));
  cv_.notify_one();
}

void CsvSink::worker()
{
  // ⭐⭐ W82：worker 用 SCHED_IDLE —— 只在 CPU 空闲时才跑，
  //   保证【永远不抢自瞄线程】（不需要 root；失败也只是不生效）
  tools::set_idle_policy();
  while (true) {
    std::string series;
    auto_aim::FrameDebug frame{};
    bool have_frame = false;
    {
      std::unique_lock lk(mtx_);
      cv_.wait(lk, [this] { return quit_ || !qf_.empty() || !qs_.empty(); });
      // ⭐ 优先清帧队列（吞吐大），也顺手带一条曲线
      if (!qf_.empty()) {
        frame = qf_.front();
        qf_.pop_front();
        have_frame = true;
      }
      if (!qs_.empty()) {
        series = std::move(qs_.front());
        qs_.pop_front();
      }
      if (!have_frame && series.empty()) {
        if (quit_) break;
        continue;
      }
    }
    // ⭐⭐ 格式化 + 落盘都在 worker 线程（原来格式化在自瞄线程）
    if (have_frame) {
      const auto s = row(frame);
      if (ff_) std::fputs(s.c_str(), ff_);
      ++rows_;
    }
    if (!series.empty() && sf_) std::fputs(series.c_str(), sf_);
  }
}

void CsvSink::close()
{
  {
    std::lock_guard lk(mtx_);
    quit_ = true;
  }
  cv_.notify_all();
  if (th_.joinable()) th_.join();
  if (ff_) { std::fclose(ff_); ff_ = nullptr; }
  if (sf_) { std::fclose(sf_); sf_ = nullptr; }
  if (shutdown_log_)
    tools::logger()->info(
      "[CsvSink] 结束：写入 {} 帧{}", rows_,
      dropped_ > 0 ? " ，丢弃 " + std::to_string(dropped_) + " 帧（超上限）" : "");
}

}  // namespace tools
