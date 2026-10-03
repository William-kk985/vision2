# archive/target_plugins —— ⭐ **冻结：`Target` 的策略槽位设计**

> **冻结于 W43。原因：不需要。**

---

## 冻的是什么

W23 把 `Target` 的 **3 个策略槽位**（虚接口 + 8 实现，550 行）迁进了本项目：

| 槽位 | 接口 | 实现 |
|---|---|---|
| 角速度估计 | `IAngularVelocityEstimator` | `EkfStateOnly` · `EkfOnly` · `VisualDiff` · `ImuFusion` |
| 过程噪声 | `IProcessNoiseAdapter` | `FixedNoise` · `NisBasedAdapter` |
| 观测滤波 | `IMeasurementFilter` | `PassthroughFilter` · `MedianFilter` |

**来源**：`Hz_Rm_Vision/src/core/tracking/target_plugins.hpp`（459 行，
带 `Requirements: 2.1 / 4.1~4.10` 的需求追溯注释）。

---

## 为什么冻结

| 理由 | 说明 |
|---|---|
| **调试已有更好的手段** | ⭐ **宏（现在是 `core/debug.hpp`，当时是 `config.hpp`）+ `test/` + `scripts/`** 已覆盖"看见内部"的需求 |
| **换算法有更简单的手段** | ⭐ 改一行 + 重编译；或直接用 `utils/wheels/` 的轮子 |
| **实测有代价** | `Target` 拷贝 **291 ns → 318 ns（+9~14%）**，`sizeof` 304 → 344 B（**同机交替 10 轮实测**） |
| ⭐ **收益不明确** | 8 个实现里**只有 2 个有真实算法差异**（ω 估计 / Q 自适应），第 3 个（中值滤波）一个开关就够 |
| **符合「谁需要谁调用」** | `Target` 在**每帧热路径**上（`Planner::plan(Target)` 按值传），不该为"可能用到"付代价 |

---

## 替代方案（已在用）

| 原来在槽位里 | 现在在哪 |
|---|---|
| 4 个角速度估计 | ⭐ **`utils/wheels/estimate/angular_velocity.hpp`**（纯类，无虚接口） |
| 2 个噪声调整 | ⭐ **`utils/wheels/estimate/process_noise.hpp`** |
| 2 个滤波 | ⭐ **`utils/wheels/filter/median_filter.hpp`** |
| **`Target`** | ⭐ **恢复同济原样**（硬编码 `v1/v2`、直读 `ekf_x()[7]`、无滤波层） |

**轮子用法**（零虚函数开销）：
```cpp
utils::wheels::VisualDiff est;
est.update(yaw, t);
double omega = est.estimate(ekf_x[7]);   // 数据不足时回退 EKF
```

---

## ⭐ 从这个设计里保留的**教训**（有价值，别丢）

1. **B13**：`NisBasedAdapter` 原实现的**上下限与基线冲突** →
   `v2=400` 但上限 300 → **NIS 变大反而把 Q 降到 300**（方向反了）。
   ⭐ **已在 `process_noise.hpp` 里修好**（`v2_max = 800`）。
2. **命名与行为不符**：`EkfOnly` 实际做**首尾中心差分**（不是纯 EKF）；
   `ImuFusion` 实际**不用 IMU**（是短期/长期差分互补）。轮子里已写清注释。
3. **中值滤波只有 20 行** —— 做成虚接口 + 2 类 + `clone()` 是**杀鸡用牛刀**。

---

## ⚠️ 注意事项

- `target_with_slots.{hpp,cpp}.txt` 是**当时接入槽位的 `Target`**（存档，**不要直接编译**）
- `target_plugins_factory.{hpp,cpp}` 是 W42 加的 yaml 入口（同步冻结）
- ⭐ 若将来确实需要运行期切换，**优先考虑**：
  1. 先用 `utils/wheels/` 的轮子在手写路径上验证
  2. 确认有收益后，再考虑 `std::variant`（值语义，拷贝不额外 `new`）而非 `unique_ptr`
