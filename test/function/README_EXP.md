# `test/function/` —— 测试 **与** ⭐ 实验（中间层）

> ⭐ **本目录有两种东西**，靠文件名一眼区分：
> | 前缀 | 是什么 | 生命周期 |
> |---|---|---|
> | `test_*.cpp` | **单一测试**（ctest 收集） | 长期 |
> | ⭐ `exp_*.cpp` | ⭐ **算法实验**（`core/debug.hpp` §二 驱动） | ⚠️ **验证完必须转正或删** |

---

## ⭐⭐⭐ 三层「测试/调试」策略（本项目）

```
┌─ 正式代码（core/…/<角色>/）──────────── 长期，**以同济为准**
│
├─ ⭐⭐ core/debug.hpp（宏 + 重新编译）──── **中间层**：算法的【临时添加 / 替换】
│
└─ test/function/ ─────────────────────── 单一测试 / 较大的改动测试
```

### ⚠️ 为什么需要中间层

直接在正式代码里手改做实验，**很容易忘记改回来**（"这个 0.7 是谁改的？"）。
放中间层 ⇒ **一行开关，可发现、可一键还原**。

### ⭐ 为什么算法**不用**运行期热插拔

| | 日志 | 算法 |
|---|---|---|
| 形态 | 运行期（`--log-off`）**+** 编译期宏 | ⭐ **只用编译期宏** |
| 理由 | 两种场景都需要 | ⭐ **最终只留一套算法**；热插拔会让"**当前跑的是哪套**"变得不确定，而算法对比最怕这个 |

⚠️ **但兼容性保留**：实验开关**默认全关**（= 完全同济行为）；
旧的 `trajectory_impl` / `detector_impl` 那类**运行期**槽位仍然在（那是"配置"，不是"实验"）。

---

## ⭐ 怎么加一个实验（3 步，**不用手改 CMake**）

### ① 写实现：`test/function/exp_<小写名字>.{hpp,cpp}`

```cpp
// exp_my_idea.hpp
#pragma once
namespace hzmir_exp {
/// ⭐ 用 constexpr 函数而不是裸露的 #ifdef —— 调用点写起来像普通函数
inline constexpr bool my_idea() {
#ifdef HZMIR_EXP_MY_IDEA
  return true;
#else
  return false;
#endif
}
inline const char * my_idea_desc() { return "一句话说清这个实验在试什么"; }
}  // namespace hzmir_exp
```

### ② 在 `core/debug.hpp` §二 加一行开关

```cpp
#define HZMIR_EXP_MY_IDEA     // 一句话说清这个实验在试什么
```

### ③ 在**相应文件**加装配入口（⚠️ **装配点**，不许散落到算法内部）

```cpp
#include "test/function/exp_my_idea.hpp"

// 用「编译期常量」分流 —— ⭐ 不是 #ifdef，宏纪律没破
if (hzmir_exp::my_idea()) {
  tools::logger()->warn("[Xxx] ⚠️ 实验模式：HZMIR_EXP_MY_IDEA");   // ⭐ 别静默
  ...实验实现...
} else {
  ...正式路径...
}
```

⭐ **CMake 会自动**（顶层 `CMakeLists.txt` 从 `core/debug.hpp` 解析）：
```
🧪 实验开关: HZMIR_EXP_MY_IDEA  →  test/function/exp_my_idea.cpp
🧪 实验模式：【已开启】
```
⇒ **定义宏 + 把 `exp_my_idea.cpp` 挂进 `hzmir_core`** —— ⚠️ **这一步原来最容易忘**（忘了就链接不到符号）。

---

## ⚠️⚠️ 实验的【生命周期】—— 这是中间层存在的意义

**结论出来后必须二选一**：

| 结果 | 怎么做 |
|---|---|
| ✅ **有效** | ⭐ **转正**：实现搬进 `core/…/<角色>/`，**删掉开关和入口** |
| ❌ **无效** | ⭐ **删除**：`exp_*.cpp` + `core/debug.hpp` 那行 + 装配入口，**一起删** |

⚠️ **不许长期挂在这儿** —— 否则它就从"实验"变成"**没人敢删的隐藏功能**"。

---

## ⭐ 关掉开关时是【真·零残留】

```
实验 ON  → nm 能在 libhzmir_core.a 里找到实验符号     ✅
实验 OFF → nm 找不到（exp_*.cpp 根本没参与编译）      ⬜
```

⭐ 因为关掉时是 **CMake 层面不加源码**，不只是 `#if` 掉 ——
**不会有"编进去了但没跑"的死代码，也没有运行期分支。**

---

## 现在的实验

| 开关 | 试什么 | 实现 |
|---|---|---|
| `HZMIR_EXP_NO_TRAD_SAVE` | 关掉传统检测器的落图（跑批省盘+省时间） | `exp_no_trad_save.{hpp,cpp}` |

⭐ 自查开着的实验：`./src/<兵种> --print-config params/robots/<兵种>.yaml` → 看 **③.5** 节。
