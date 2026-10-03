# omniperception —— 已冻结（2026-09-30）

**来源**：同济 `tasks/omniperception/`（`perceptron` + `decider` + `detection.hpp`）

**冻结原因**（见 `docs/autoaim_compare/09` §7.5.2）：
⭐ **本项目的哨兵和步兵一样只有一个摄像头** → 4 相机全向感知用不上。

| 角色 | 打符…不对，是 omniperception 里的位置 | 与相机数有关? |
|---|---|:---:|
| 4 相机 + 4 YOLO 并行 | `perceptron.cpp` | ✅ 有关 → 归档 |
| `decide()` × 3 重载 / `delta_angle` / `sort` | `decider.cpp` | ✅ 有关 → 归档 |

**⭐ 但已有 4 项横切能力被捞出来**（见 `09` §7.5.4）：

| # | 能力 | 去向 | 状态 |
|---|---|---|---|
| 1 | `armor_filter` 5 条射击过滤 | `core/auto_aim/tracker/filter.cpp` | ⏳ W7 |
| 2 | **无敌过滤** | `core/types.hpp` + `core/auto_aim/shooter/` | ⏳ W8 |
| 3 | `get_auto_aim_target` 集火指令 | `core/auto_aim/tracker/` | ⏳ W7 |
| 4 | `set_priority` 4 个 PriorityMap | `core/auto_aim/tracker/priority.hpp` | ⏳ W7 |
| — | `DetectionResult`（数据结构） | **`core/types.hpp`** | ✅ **W4 已完成** |

**复活步骤**：若要重新支持多相机哨兵 →
 1. 把 `perceptron.cpp` / `decider.cpp` 搬到 `core/omniperception/{detector,pipeline}/`
 2. `decider` 的横切能力改为复用 `core/` 里已抽出的实现（不要重复）
 3. 在 `config.hpp` 打开 `ENABLE_OMNIPERCEPTION`（它需要 `HAS_USBCAMERA`，互斥检查会验证）
  ⚠️ **W96 起 `config.hpp` 已删除** —— 若要恢复本模块，开关应加到 **`core/debug.hpp`**（开关与实验总控）

**验证记录**：未验证（不参与构建）
