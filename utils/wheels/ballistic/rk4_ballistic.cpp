/**
 * @file utils/wheels/ballistic/rk4_ballistic.cpp
 * @brief 弹道解算（RK4 含阻力）—— 按原实现移植，注释重写
 */
#include "utils/wheels/ballistic/rk4_ballistic.hpp"

#include <algorithm>
#include <limits>

namespace tools
{

namespace
{
// ── RM 官方弹丸参数（原样保留）──
constexpr double SMALL_MASS = 0.0032;      // 3.2 g
constexpr double SMALL_DIAMETER = 0.017;   // 17 mm
constexpr double SMALL_CD = 0.47;          // 球形阻力系数

constexpr double BIG_MASS = 0.044;         // 44 g（高尔夫球）
constexpr double BIG_DIAMETER = 0.042;     // 42 mm
constexpr double BIG_CD = 0.25;            // 凹坑高尔夫球阻力系数
}  // namespace

BallisticSolver::BallisticSolver(BulletType bullet_type)
{
  switch (bullet_type) {
    case BulletType::BIG_42MM:
      mass_ = BIG_MASS;
      diameter_ = BIG_DIAMETER;
      drag_coeff_ = BIG_CD;
      break;
    case BulletType::SMALL_17MM:
    default:
      mass_ = SMALL_MASS;
      diameter_ = SMALL_DIAMETER;
      drag_coeff_ = SMALL_CD;
      break;
  }
  update_k();
}

void BallisticSolver::set_bullet_params(double mass, double diameter, double drag_coeff)
{
  mass_ = mass;
  diameter_ = diameter;
  drag_coeff_ = drag_coeff;
  update_k();
}

void BallisticSolver::update_k()
{
  // k = 0.5 · ρ · Cd · A / m ，A = π·(d/2)²
  const double area = M_PI * (diameter_ / 2) * (diameter_ / 2);
  k_ = 0.5 * RHO * drag_coeff_ * area / mass_;
}

BallisticResult BallisticSolver::solve(
  double v0, double distance, double height, bool use_air_resistance) const
{
  return use_air_resistance ? solve_rk4(v0, distance, height)
                            : solve_simple(v0, distance, height);
}

BallisticResult BallisticSolver::solve_simple(double v0, double distance, double height) const
{
  BallisticResult result;
  result.iterations = 1;

  // 抛物线模型：h = d·tanθ -g·d²/(2v0²cos²θ)
  // 令 t = tanθ, a = g·d²/(2v0²)  →  a·t² -d·t + (a + h) = 0
  const double a = G * distance * distance / (2 * v0 * v0);
  const double b = -distance;
  const double c = a + height;
  const double delta = b * b - 4 * a * c;

  if (delta < 0) return result;   // 无解（打不到）

  const double tan_1 = (-b + std::sqrt(delta)) / (2 * a);
  const double tan_2 = (-b - std::sqrt(delta)) / (2 * a);
  const double pitch_1 = std::atan(tan_1);
  const double pitch_2 = std::atan(tan_2);
  const double t_1 = distance / (v0 * std::cos(pitch_1));
  const double t_2 = distance / (v0 * std::cos(pitch_2));

  // 取飞行时间短的解（低伸弹道）
  const bool use_first = t_1 < t_2;
  result.solved = true;
  result.pitch = use_first ? pitch_1 : pitch_2;
  result.fly_time = use_first ? t_1 : t_2;
  result.drop = 0;   // 无阻力模型不算额外下坠
  return result;
}

BallisticResult BallisticSolver::solve_rk4(double v0, double distance, double height) const
{
  BallisticResult result;

  // 用无阻力解做初值 + 定义搜索上界
  const auto simple = solve_simple(v0, distance, height);
  if (!simple.solved) return result;

  double pitch_low = 0.0;
  double pitch_high = M_PI / 4;      // 上界 45°
  double pitch_mid = simple.pitch;

  double y_low, y_high, t_low, t_high;
  if (!simulate_trajectory(v0, pitch_low, distance, y_low, t_low))
    y_low = -std::numeric_limits<double>::max();
  if (!simulate_trajectory(v0, pitch_high, distance, y_high, t_high))
    y_high = std::numeric_limits<double>::max();

  // 二分加仰角，直到落点高度命中
  for (int iter = 0; iter < MAX_ITER; ++iter) {
    result.iterations = iter + 1;

    double y_mid = 0, t_mid = 0;
    if (!simulate_trajectory(v0, pitch_mid, distance, y_mid, t_mid)) {
      pitch_low = pitch_mid;                       // 打不到 → 抬仰角
      pitch_mid = (pitch_low + pitch_high) / 2;
      continue;
    }

    const double error = y_mid - height;
    if (std::abs(error) < TOL) {
      result.solved = true;
      result.pitch = pitch_mid;
      result.fly_time = t_mid;
      // 相对无阻力模型的额外下坠
      const double simple_y =
        distance * std::tan(simple.pitch) - 0.5 * G * simple.fly_time * simple.fly_time;
      result.drop = simple_y - y_mid;
      return result;
    }

    if (error > 0)
      pitch_high = pitch_mid;   // 打高了
    else
      pitch_low = pitch_mid;    // 打低了
    pitch_mid = (pitch_low + pitch_high) / 2;
  }

  // 未收敛：返回最后一次结果（solved 仍置 true，靠 iterations == MAX_ITER 判别）
  double y_final = 0, t_final = 0;
  if (simulate_trajectory(v0, pitch_mid, distance, y_final, t_final)) {
    result.solved = true;
    result.pitch = pitch_mid;
    result.fly_time = t_final;
    result.drop = height - y_final;
  }
  return result;
}

bool BallisticSolver::simulate_trajectory(
  double v0, double pitch, double target_x, double & final_y, double & fly_time) const
{
  // state = [x, y, vx, vy]（与原实现一致）
  std::array<double, 4> state = {0.0, 0.0, v0 * std::cos(pitch), v0 * std::sin(pitch)};

  double t = 0.0;
  while (t < MAX_TIME) {
    if (state[0] >= target_x) {
      // 线性插值回推到恰好 target_x
      const double dt_back = (state[0] - target_x) / state[2];
      final_y = state[1] - state[3] * dt_back;
      fly_time = t - dt_back;
      return true;
    }
    if (state[1] < GROUND_Y) return false;   // ⭐ 落地即停（同济 h_solver 的做法）
    state = rk4_step(state, dt_);
    t += dt_;
  }
  return false;   // 超时
}

std::array<double, 4> BallisticSolver::rk4_step(
  const std::array<double, 4> & state, double dt) const
{
  const auto k1 = derivatives(state);

  std::array<double, 4> s2;
  for (int i = 0; i < 4; ++i) s2[i] = state[i] + 0.5 * dt * k1[i];
  const auto k2 = derivatives(s2);

  std::array<double, 4> s3;
  for (int i = 0; i < 4; ++i) s3[i] = state[i] + 0.5 * dt * k2[i];
  const auto k3 = derivatives(s3);

  std::array<double, 4> s4;
  for (int i = 0; i < 4; ++i) s4[i] = state[i] + dt * k3[i];
  const auto k4 = derivatives(s4);

  std::array<double, 4> out;
  for (int i = 0; i < 4; ++i)
    out[i] = state[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  return out;
}

std::array<double, 4> BallisticSolver::derivatives(const std::array<double, 4> & state) const
{
  // state = [x, y, vx, vy]
  const double vx = state[2], vy = state[3];
  const double v = std::sqrt(vx * vx + vy * vy);

  // 阻力 a = -k·|v|·v（k 已含 1/m）
  const double ax = -k_ * v * vx;
  const double ay = -G - k_ * v * vy;
  return {vx, vy, ax, ay};
}

std::vector<std::pair<double, double>> BallisticSolver::get_trajectory_points(
  double v0, double pitch, double target_x, int num_points) const
{
  std::vector<std::pair<double, double>> points;
  if (num_points < 2) return points;
  points.reserve(num_points);

  std::array<double, 4> state = {0.0, 0.0, v0 * std::cos(pitch), v0 * std::sin(pitch)};

  const double x_step = target_x / (num_points - 1);
  double t = 0.0;
  int idx = 0;
  double next_x = 0.0;

  points.emplace_back(0.0, 0.0);
  ++idx;
  next_x = x_step;

  while (t < MAX_TIME && idx < num_points) {
    if (state[0] >= next_x) {
      const double dt_back = (state[0] - next_x) / state[2];
      points.emplace_back(next_x, state[1] - state[3] * dt_back);
      ++idx;
      next_x = idx * x_step;
    }
    if (state[1] < GROUND_Y) break;
    state = rk4_step(state, dt_);
    t += dt_;
  }
  return points;
}

}  // namespace tools
