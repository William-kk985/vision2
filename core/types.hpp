/**
 * @file core/types.hpp
 * @brief **数据契约层** —— 模块之间「说什么」
 *
 * 来源（🔀 合并）：
 *   - `tasks/auto_aim/armor.hpp`   -> namespace auto_aim
 *   - `tools/trajectory.hpp`       -> namespace tools
 *   - `io/command.hpp`             -> namespace io
 *
 * ⚠️ **零依赖纪律**：只允许 Eigen / opencv2 / STL。
 *    验证：`grep "#include" core/types.hpp`
 *
 * ⭐ 命名空间保持同济原样（`auto_aim` / `tools` / `io`）→ 所有调用点零改动。
 */
#ifndef HZMIR_CORE_TYPES_HPP
#define HZMIR_CORE_TYPES_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <list>
#include <string>
#include <tuple>
#include <vector>

// ═══════════════════════════════════════════════════════════════
// 来自 tasks/auto_aim/armor.hpp
// ═══════════════════════════════════════════════════════════════
namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType
{
  big,
  small
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

struct Lightbar
{
  // ⭐ E2 家族修复：POD 成员全部给默认值。
  //   原来 `Lightbar() {};` 只声明不初始化 → 默认构造后读到的是**垃圾值**，
  //   与 C18（Armor::priority）是同一类问题。
  std::size_t id = 0;
  Color color = Color::red;
  cv::Point2f center{0, 0}, top{0, 0}, bottom{0, 0}, top2bottom{0, 0};
  std::vector<cv::Point2f> points;
  double angle = 0, angle_error = 0, length = 0, width = 0, ratio = 0;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() = default;   // ⭐ 现在可以安全 = default
};

struct Armor
{
  // ═══════════════════════════════════════════════════════════════
  // ⭐⭐ E2 根因修复：**原来没有默认构造函数**
  //
  // `Armor` 声明了 5 个构造函数 → 编译器不生成默认构造 →
  // 任何想「持有 Armor」的容器/结构体都写不出来，只能靠**聚合初始化**绕，
  // 于是出现了 `Armor armor{0, 0, 0.0f, cv::Rect(), {}, cv::Point2f()}`
  // —— 但第 2 个成员是 `Lightbar`，**不能用 `0` 构造** → 那段代码根本非法。
  //
  // 修法：① 全部 POD 成员给默认值（与 C18 同一原则）
  //       ② `Armor() = default` → 容器友好（如 `tools::TIResult<Armor>`）
  // ═══════════════════════════════════════════════════════════════
  Color color = Color::red;
  Lightbar left, right;     //used to be const
  cv::Point2f center{0, 0};       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm{0, 0};  // 归一化坐标
  std::vector<cv::Point2f> points;

  double ratio = 0;              // 两灯条的中点连线与长灯条的长度之比
  double side_ratio = 0;         // 长灯条与短灯条的长度之比
  double rectangular_error = 0;  // 灯条和中点连线所成夹角与π/2的差值

  ArmorType type = ArmorType::big;
  ArmorName name = ArmorName::not_armor;
  ArmorPriority priority = ArmorPriority::fifth;  // ⭐ C18 修复：默认值（原为未初始化）
  int class_id = 0;
  cv::Rect box;
  cv::Mat pattern;
  double confidence = 0;   // ⭐ E2 家族修复
  bool duplicated = false; // ⭐ E2 家族修复

  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();  // 单位：m
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();   // 单位：m
  Eigen::Vector3d ypr_in_gimbal = Eigen::Vector3d::Zero();  // 单位：rad
  Eigen::Vector3d ypr_in_world = Eigen::Vector3d::Zero();   // 单位：rad
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();   // 球坐标系

  double yaw_raw = 0;  // rad

  Armor() = default;   // ⭐⭐ E2 修复：容器友好
  Armor(const Lightbar & left, const Lightbar & right);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

}  // namespace auto_aim

// ═══════════════════════════════════════════════════════════════
// ⭐ 无敌状态（从 omniperception::Decider::get_invincible_armor 捞出）
//
// 原实现：`invincible_armor_.push_back(ArmorName(id - 1));`
//   ⚠️ id-1 隐式减法脆弱：ArmorName 枚举顺序一改，映射全错且编译器不报错
// 现实现：**显式映射表** + **1 字节位掩码**（零分配、零序列化成本）
// ⚠️ 待与电控核对：本表按「6=sentry / 7=outpost / 8=base」，
//    而 RM 标准 ID 是「6=空中 / 7=哨兵」—— 错了就永远过滤不到哨兵
// ═══════════════════════════════════════════════════════════════
namespace auto_aim
{
inline constexpr std::array<std::pair<int, ArmorName>, 8> ENEMY_ID_TO_ARMOR = {{
  {1, ArmorName::one},   {2, ArmorName::two},     {3, ArmorName::three}, {4, ArmorName::four},
  {5, ArmorName::five},  {6, ArmorName::sentry},  {7, ArmorName::outpost}, {8, ArmorName::base},
}};

/// @brief 无敌状态位掩码：bit(i-1) = 敌人 i 处于无敌
struct InvincibleMask
{
  uint8_t mask = 0;

  void set(int enemy_id)
  {
    if (enemy_id >= 1 && enemy_id <= 8) mask |= static_cast<uint8_t>(1u << (enemy_id - 1));
  }
  void clear() { mask = 0; }

  bool has(ArmorName n) const
  {
    for (const auto & [id, name] : ENEMY_ID_TO_ARMOR)
      if (name == n) return (mask >> (id - 1)) & 1u;
    return false;
  }

  /// @brief 从下位机/ROS 来的 id 列表构造
  static InvincibleMask from_ids(const std::vector<int8_t> & ids)
  {
    InvincibleMask m;
    for (auto id : ids) m.set(static_cast<int>(id));
    return m;
  }
};

/// @brief enemy_id -> ArmorName（未知 id 返回 not_armor，不越界）
inline ArmorName id_to_armor(int enemy_id)
{
  for (const auto & [id, name] : ENEMY_ID_TO_ARMOR)
    if (id == enemy_id) return name;
  return ArmorName::not_armor;
}
}  // namespace auto_aim

// ═══════════════════════════════════════════════════════════════
// 来自 tools/trajectory.hpp
// ═══════════════════════════════════════════════════════════════
namespace tools
{
struct Trajectory
{
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为正

  // 不考虑空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  Trajectory(const double v0, const double d, const double h);
};

}  // namespace tools

// ═══════════════════════════════════════════════════════════════
// 来自 io/command.hpp
// ═══════════════════════════════════════════════════════════════
namespace io
{
struct Command
{
  // ⭐⭐ E2 家族修复：POD 成员全部给默认值
  //   原为 `bool control; bool shoot; double yaw; double pitch;` —— **全未初始化**。
  //   实测踩到：`uav.cpp` 里 `io::Command buff_command;` 若两个档位分支都不匹配，
  //   就会把**未初始化的值**发给下位机（同 C18/E2 一类）。
  bool control = false;
  bool shoot = false;
  double yaw = 0;
  double pitch = 0;
  double horizon_distance = 0;  //无人机专有
};

}  // namespace io


// ═══════════════════════════════════════════════════════════════
// ⭐ 从 tasks/omniperception/detection.hpp 提升到契约层
//
// 原因：原 `auto_aim::Tracker` 为了这一个纯数据结构，去 include 了
//       `tasks/omniperception/perceptron.hpp` —— 连带拉进
//       `io::USBCamera` / `Decider` / `thread_pool` 一大堆依赖，
//       形成 **功能组 A 依赖功能组 B** 的架构违规。
//       它是「多相机的一次检测结果」，属于数据契约，不属于任何功能组。
// ⭐ 同时修：delta_yaw/delta_pitch 加默认值（原为未初始化 POD 成员）
// ═══════════════════════════════════════════════════════════════
namespace auto_aim
{
struct DetectionResult
{
  std::list<Armor> armors;
  std::chrono::steady_clock::time_point timestamp;
  double delta_yaw = 0;    // rad  ⭐ 加默认值
  double delta_pitch = 0;  // rad  ⭐ 加默认值
};
}  // namespace auto_aim

// 兼容别名：原调用点写的是 omniperception::DetectionResult
namespace omniperception
{
using DetectionResult = auto_aim::DetectionResult;
}  // namespace omniperception

#endif  // HZMIR_CORE_TYPES_HPP
