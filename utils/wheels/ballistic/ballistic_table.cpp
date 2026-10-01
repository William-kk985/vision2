#include "utils/wheels/ballistic/ballistic_table.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace tools
{

int BallisticTable::i_v0(double v0) const
{
  const int n = v0_axis_.size();
  int i = static_cast<int>((v0 - v0_axis_.min) / v0_axis_.step);
  return std::clamp(i, 0, n - 1);
}

int BallisticTable::i_d(double d) const
{
  const int n = dist_axis_.size();
  int i = static_cast<int>((d - dist_axis_.min) / dist_axis_.step);
  return std::clamp(i, 0, n - 1);
}

double BallisticTable::build(
  const BallisticSolver & solver, const BaselineFn & baseline, const TableAxis & v0_axis,
  const TableAxis & dist_axis, double height)
{
  if (v0_axis.step <= 0 || dist_axis.step <= 0)
    throw std::runtime_error("[BallisticTable] 轴 step 必须为正");

  v0_axis_ = v0_axis;
  dist_axis_ = dist_axis;

  const int nv = v0_axis_.size(), nd = dist_axis_.size();
  data_.assign(static_cast<size_t>(nv) * nd, BallisticCorrection{});

  const auto t0 = std::chrono::steady_clock::now();
  int bad = 0;
  for (int iv = 0; iv < nv; ++iv) {
    const double v0 = v0_axis_.min + iv * v0_axis_.step;
    for (int id = 0; id < nd; ++id) {
      const double d = dist_axis_.min + id * dist_axis_.step;

      // 基线（注入的无阻力解析解）—— 与在线路径用**同一份**基线，保证修正量定义一致
      const BallisticResult base = baseline(v0, d, height);
      BallisticCorrection & c = data_[static_cast<size_t>(iv) * nd + id];
      if (!base.solved) {
        c = BallisticCorrection{};   // 无解点无修正
        ++bad;
        continue;
      }

      const auto r = solver.solve(v0, d, height, true);
      if (!r.solved) {
        ++bad;
        continue;
      }
      c.dpitch = r.pitch - base.pitch;
      c.dt_fly = r.fly_time - base.fly_time;
    }
  }

  const double secs =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  if (bad > 0)
    std::fprintf(stderr, "[BallisticTable] %d/%d 个网格点无解或未收敛\n", bad, nv * nd);
  return secs;
}

BallisticCorrection BallisticTable::correction(double v0, double distance) const
{
  if (data_.empty()) return {};

  const double fv = (v0 - v0_axis_.min) / v0_axis_.step;
  const double fd = (distance - dist_axis_.min) / dist_axis_.step;

  const int iv0 = std::clamp(static_cast<int>(std::floor(fv)), 0, v0_axis_.size() - 1);
  const int id0 = std::clamp(static_cast<int>(std::floor(fd)), 0, dist_axis_.size() - 1);
  const int iv1 = std::min(iv0 + 1, v0_axis_.size() - 1);
  const int id1 = std::min(id0 + 1, dist_axis_.size() - 1);

  const double tv = std::clamp(fv - iv0, 0.0, 1.0);
  const double td = std::clamp(fd - id0, 0.0, 1.0);

  auto lerp = [](double a, double b, double t) { return a + (b - a) * t; };

  // 双线性
  BallisticCorrection out;
  out.dpitch = lerp(
    lerp(at(iv0, id0).dpitch, at(iv0, id1).dpitch, td),
    lerp(at(iv1, id0).dpitch, at(iv1, id1).dpitch, td), tv);
  out.dt_fly = lerp(
    lerp(at(iv0, id0).dt_fly, at(iv0, id1).dt_fly, td),
    lerp(at(iv1, id0).dt_fly, at(iv1, id1).dt_fly, td), tv);
  return out;
}

bool BallisticTable::save(const std::string & path) const
{
  FILE * f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
  const double hdr[4] = {
    v0_axis_.min, v0_axis_.step, dist_axis_.min, dist_axis_.step};
  const int n[2] = {v0_axis_.size(), dist_axis_.size()};
  std::fwrite(hdr, sizeof(double), 4, f);
  std::fwrite(n, sizeof(int), 2, f);
  // 用 float 存（修正量动态范围窄，省一半空间）
  for (const auto & c : data_) {
    const float p[2] = {static_cast<float>(c.dpitch), static_cast<float>(c.dt_fly)};
    std::fwrite(p, sizeof(float), 2, f);
  }
  std::fclose(f);
  return true;
}

bool BallisticTable::load(const std::string & path)
{
  FILE * f = std::fopen(path.c_str(), "rb");
  if (!f) return false;
  double hdr[4];
  int n[2];
  if (std::fread(hdr, sizeof(double), 4, f) != 4 || std::fread(n, sizeof(int), 2, f) != 2) {
    std::fclose(f);
    return false;
  }
  v0_axis_ = {hdr[0], hdr[0] + hdr[1] * (n[0] - 1), hdr[1]};
  dist_axis_ = {hdr[2], hdr[2] + hdr[3] * (n[1] - 1), hdr[3]};

  data_.resize(static_cast<size_t>(n[0]) * n[1]);
  for (auto & c : data_) {
    float p[2];
    if (std::fread(p, sizeof(float), 2, f) != 2) { std::fclose(f); return false; }
    c.dpitch = p[0];
    c.dt_fly = p[1];
  }
  std::fclose(f);
  return true;
}

double BallisticTable::out_of_range_ratio(double v0, double d) const
{
  if (data_.empty()) return 1.0;
  const bool in = (v0 >= v0_axis_.min && v0 <= v0_axis_.max && d >= dist_axis_.min &&
                   d <= dist_axis_.max);
  return in ? 0.0 : 1.0;
}

}  // namespace tools
