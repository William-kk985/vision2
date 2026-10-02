# 用 MVS 诊断海康相机 —— 看什么

> ⚠️ **MVS 是诊断工具，不是解决方案。** 看完**必须完全关闭**再跑我们的程序 ——
> 海康 USB 相机**不独占**（SDK 文档：`nAccessMode` 对 USB 无效），
> MVS 和我们的程序能**同时 OpenDevice**，但**只有一个能取到流**。

---

## 打开

```bash
/opt/MVS/bin/MVS.sh
# 或者
/opt/MVS/bin/MVS
```

---

## ⭐ 第 1 件事：**能不能出图**（最重要）

在设备列表里**双击相机** → 点 **`开始采集`**（或工具栏的 ▶）

| 结果 | 结论 | 下一步 |
|---|---|---|
| ✅ **能出图** | 相机 / USB / 驱动**全都正常** | ⭐ **我们的问题在软件层** → 关掉 MVS 再跑程序；若仍失败，看第 2~6 项 |
| ❌ **不出图 / 报错** | ⚠️ **硬件或 USB 层问题** | 换 USB3 线 / 换口 / 看第 2~6 项 |

---

## ⭐ 第 2~7 项：进 `Feature` 面板（右侧）逐项看

按 Ctrl+F 或点右侧 `Features` 树，找到并**记录**这些值：

| # | 节点（Features 树里） | 期望值 | 说明 |
|---|---|---|---|
| 2 | `AcquisitionControl` → **`TriggerMode`** | **`Off`** | ⚠️ 若是 `On` → 程序永远等不到帧（我们的驱动已强制设 Off） |
| 3 | `AcquisitionControl` → **`AcquisitionFrameRate`** | 30 左右 | 我们的 yaml `fps: 30`；原来硬编码 150 |
| 4 | `AcquisitionControl` → **`ResultingFrameRate`** | 接近上面那个 | ⭐ 实际能跑到的帧率 |
| 5 | `ImageFormatControl` → **`Width` / `Height`** | 1280×720（你的日志） | ⭐ 决定带宽 |
| 6 | `ImageFormatControl` → **`PixelFormat`** | `BayerRG8` 之类 | 1 字节/像素 |
| 7 | `DeviceControl` → **`DeviceLinkSpeed`** ⭐ | **`SuperSpeed`**（USB3） | ⚠️ 若是 `HighSpeed` = **USB2** → 换线/换口 |

### ⭐ 最有价值的两项

**`TriggerMode`** 和 **`DeviceLinkSpeed`**。

- `TriggerMode = On` → 相机在等外部触发 → **永远没帧**（`0x80000007`）
- `DeviceLinkSpeed = HighSpeed` → **跑在 USB2** → 工业相机基本不可用

---

## ⭐ 第 8 项：看时间戳/帧率统计

MVS 底部或右侧有 **帧率 / 丢帧统计**：

| 现象 | 含义 |
|---|---|
| 帧率稳定、丢帧 0 | ✅ 硬件没问题 |
| ⚠️ **帧率 0 / 一直不涨** | ⚠️ **和我们程序一样的症状** → 硬件/USB 问题 |
| ⚠️ 丢帧很多 | USB 带宽或线材问题 |

---

## 🔴 看完之后（关键！）

```bash
# ⭐ 完全关闭 MVS（GUI 关掉可能还有残留进程）
pkill -f MVS
pgrep -af MVS || echo "✅ MVS 已完全关闭"

# 然后跑我们的程序（run.sh 现在会自动检查 MVS 有没有关干净）
tools/scripts/run.sh infantry
```

---

## 对照表：MVS 结果 → 结论

| MVS 能出图？ | DeviceLinkSpeed | 结论 |
|---|---|---|
| ✅ | SuperSpeed | ⭐ **硬件全好** → 问题在**抢流**（MVS 没关干净）或我们的代码 |
| ✅ | ⚠️ HighSpeed | 能出图但带宽紧 → 降 fps / 换线 |
| ❌ | 任意 | ⚠️ **硬件/USB/驱动问题** → 换线换口，或 `tools/scripts/camera-reset.sh` |

---

## ⭐ 顺便：我们程序侧对应的日志

跑我们程序时，这几行和 MVS 的 2~7 项是**一一对应**的：
```
[HikRobot] 实际曝光 = 2000 µs          ← MVS: ExposureTime
[HikRobot] 实际增益 = 16.0 dB          ← MVS: Gain
[HikRobot] 实际帧率 = 30.0 fps         ← MVS: ResultingFrameRate
[HikRobot] TriggerMode = 0（0=Off）    ← MVS: TriggerMode
[HikRobot] 分辨率 = 1280×720，需带宽 ≈ 26 MB/s   ← MVS: Width/Height/PixelFormat
```

⭐ **如果 MVS 里看到的值和上面不一样** → 说明**相机被 MVS 改过**，
   ⚠️ 而我们的驱动**只在 `capture_start()` 时设一次**，之后 MVS 改了就会不一致。

⭐ **如果 MVS 里一切正常但我们的程序拿不到帧** → 那 99% 是**抢流**（MVS 进程没关干净）。
