/**
 * @file core/auto_aim/trajectory/table.cpp
 * @brief 弹道实现③：**离线 RK4 建表 + 在线查表**（W12 结论的落地）
 *
 * ```
 *   pitch(v0,d,h)  =  ideal(v0,d,h)  +  Δpitch(v0,d)      ← Δ 由离线 RK4 建表
 *   t_fly(v0,d,h)  =  ideal_fly(v0,d) +  Δt_fly(v0,d)
 * ```
 * ⭐ 拿到 **RK4 的精度 + ideal 的速度**（W12 实测 rk4 要 1467 µs，ideal 只要 0.03 µs）。
 */
#include <memory>
#include <stdexcept>

#include "utils/log/logger.hpp"

#include "core/auto_aim/trajectory/trajectory.hpp"
#include "utils/wheels/ballistic/ballistic_table.hpp"

namespace auto_aim
{

namespace
{
/// 基线：把 `tools::Trajectory`（无阻力解析解）包成轮子的 `BallisticResult`
tools::BallisticResult baseline_fn(double v0, double d, double h)
{
  const tools::Trajectory t(v0, d, h);
  tools::BallisticResult r;
  r.solved = !t.unsolvable;
  r.pitch = t.pitch;
  r.fly_time = t.fly_time;
  return r;
}

class TableTrajectory : public ITrajectory
{
public:
  TableTrajectory(bool big, const std::string & table_path)
  : solver_(big ? tools::BulletType::BIG_42MM : tools::BulletType::SMALL_17MM)
  {
    // ⭐ 表文件路径：显式给就用给的；否则按弹型取默认名
    const std::string path = table_path.empty()
                               ? (big ? "params/ballistic_table_42.bin"
                                      : "params/ballistic_table_17mm.bin")
                               : table_path;

    // ⭐⭐⭐ 快速路径：有表文件 → 直接载入（**毫秒级**，赛场就该走这条）
    if (table_.load(path)) {
      loaded_from_file_ = true;
      tools::logger()->info(
        "[TableTrajectory] 载入 {} （{} 项，内存 {} B）—— 启动零建表", path, table_.entries(),
        table_.bytes());
      return;
    }

    // ⚠️ 慢路径：首次运行才建表（实测 17mm 约 12~16 s），**建完立刻存盘**
    tools::TableAxis v0a{10.0, 25.0, 0.5};     // 31 点
    tools::TableAxis da{1.0, 15.0, 0.1};       // 141 点
    tools::logger()->warn(
      "[TableTrajectory] {} 不存在 → 开始建表（预计 10~20 s，仅首次；建完会存盘）", path);
    build_secs_ = table_.build(solver_, baseline_fn, v0a, da, 0.0);
    if (table_.save(path))
      tools::logger()->info(
        "[TableTrajectory] 建表完成 {:.1f} s，已存 {} —— 下次启动将直接载入", build_secs_, path);
    else
      tools::logger()->warn("[TableTrajectory] 建表完成但存盘失败（{} 不可写）", path);
  }

  tools::Trajectory solve(double v0, double d, double h) const override
  {
    tools::Trajectory t(v0, d, h);          // ⭐ 基线：处理任意 h 与 unsolvable
    if (t.unsolvable || table_.empty()) return t;

    const auto c = table_.correction(v0, d);   // ⭐ 双线性查表（纳秒级）
    t.pitch += c.dpitch;
    t.fly_time += c.dt_fly;
    return t;
  }

  const char * name() const override { return "table"; }

  double build_secs() const { return build_secs_; }
  size_t table_bytes() const { return table_.bytes(); }
  bool loaded_from_file() const { return loaded_from_file_; }

private:
  tools::BallisticSolver solver_;
  tools::BallisticTable table_;
  double build_secs_ = 0;
  bool loaded_from_file_ = false;
};
}  // namespace

std::unique_ptr<ITrajectory> make_table_trajectory(bool big, const std::string & table_path)
{
  return std::make_unique<TableTrajectory>(big, table_path);
}

}  // namespace auto_aim
