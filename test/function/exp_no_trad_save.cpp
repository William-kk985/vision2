/**
 * @file test/function/exp_no_trad_save.cpp
 * @brief ⭐ 实验实现：见 `exp_no_trad_save.hpp` 的说明
 *
 * ⚠️ 本文件**只在 `HZMIR_EXP_NO_TRAD_SAVE` 定义时才参与编译**
 *   （由顶层 `CMakeLists.txt` 从 `core/debug.hpp` 自动解析）。
 *   ⇒ 关掉开关时它是**真·零残留**（不在构建里，符号不存在）。
 */
#include "test/function/exp_no_trad_save.hpp"

#include "utils/log/logger.hpp"

namespace hzmir_exp
{

/// @brief 实验的"装配副作用"：启动时警告一句（⭐ 免得忘了自己开着实验）
///
/// ⚠️ 为什么需要它：一个实验如果**静默生效**，跑出来的数据就会被当成
///   正式行为解读 —— 这正是本层最该避免的事（"静默是敌人"）。
struct NoTradSaveRegistrar
{
  NoTradSaveRegistrar()
  {
    tools::logger()->warn(
      "[EXP] ⚠️ 实验模式：HZMIR_EXP_NO_TRAD_SAVE —— {}", no_trad_save_desc());
    tools::logger()->warn(
      "[EXP] ⚠️ 这**不是**同济默认行为；结论出来后请【转正】或【删除】"
      "（见 core/debug.hpp §二 的生命周期规则）");
  }
};

/// ⭐ 静态实例：加载本实验的 .cpp 时打一次警告
///   （被 `hzmir_core` 链接 ⇒ 程序启动即生效）
static const NoTradSaveRegistrar g_registrar;

}  // namespace hzmir_exp
