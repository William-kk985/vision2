/**
 * @file test/function/exp_no_trad_save.hpp
 * @brief ⭐ 实验：**关掉传统检测器的落图**（`HZMIR_EXP_NO_TRAD_SAVE`）
 *
 * ## 这是个"实验"而不是"功能"，为什么
 * `Detector::save()`（传统检测器）目前是**无条件**的：
 * ```
 * detector.cpp  check_name():  if (name_ok && !confidence_ok) save(armor);  // 置信度不过
 * detector.cpp  check_type():  if (!name_ok)                { ...; save(); } // 类型异常
 * ```
 * 注释写着"保存不确定/异常的图案，用于分类器迭代" —— **意图是合理的**，
 * 但它造成两个实际问题：
 *   · ⚠️ **跑批调参时持续落 JPEG**（一次录像回放能落几十上百张）
 *   · ⚠️ 而**三个 YOLO 版本的同类调用是注释掉的** ⇒ 传统/YOLO **行为不一致**
 *
 * ## 为什么不做成正式开关
 * 它改变的是**同济继承的行为**，而"最终只留一套算法" ⇒
 * ⚠️ **要么验证完转正（给 `Detector` 加个构造参数），要么放弃**。
 * 在那之前放这一层，**一行开关、可发现、可一键还原**。
 *
 * ## 用法
 * 在 `core/debug.hpp` §二 取消注释：
 * ```cpp
 * #define HZMIR_EXP_NO_TRAD_SAVE
 * ```
 * 然后重编。⚠️ **CMake 会自动**定义该宏并把本文件的 `.cpp` 挂进构建。
 */
#ifndef HZMIR_TEST_FUNCTION_EXP_NO_TRAD_SAVE_HPP
#define HZMIR_TEST_FUNCTION_EXP_NO_TRAD_SAVE_HPP

#include <string>

namespace hzmir_exp
{

/// @brief 实验是否生效（⭐ 编译期常量，关掉时调用点会被优化掉）
inline constexpr bool no_trad_save()
{
#ifdef HZMIR_EXP_NO_TRAD_SAVE
  return true;
#else
  return false;
#endif
}

/// @brief 实验一句话说明（用于启动日志 / `--print-config`）
inline const char * no_trad_save_desc()
{
  return "关掉传统检测器的落图（check_name/check_type 里的 save()）";
}

}  // namespace hzmir_exp

#endif  // HZMIR_TEST_FUNCTION_EXP_NO_TRAD_SAVE_HPP
