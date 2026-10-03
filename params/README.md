# `params/` —— 配置（⚠️ **两层，别放错**）

> ⭐ **判据**：**按「谁的东西」分，不按「什么类型」分。**
> | 目录 | 放谁的东西 | 什么时候动它 |
> |---|---|---|
> | ⭐ **`cameras/`** | **品牌 / 设备**级的 | **换相机品牌**时 |
> | ⭐ **`robots/`** | **兵种**的全部配置 | **调参**时（这是你 99% 会改的） |

---

## 📁 结构

```
params/
├── cameras/                 ⭐ 只放【品牌 / 设备】级（换相机才动）
│   ├── hikrobot.yaml          海康：VID:PID · PixelFormat · 白平衡 · 通用 camera_params
│   ├── mindvision.yaml        迈德威视：gamma
│   └── usbcamera.yaml         USB 摄像头
│
├── robots/                  ⭐ 兵种的【全部】配置（含它自己的相机参数）
│   ├── infantry.yaml          步兵：hikrobot · 曝光 10ms · gain 16
│   ├── hero.yaml              英雄：hikrobot · 曝光 2ms
│   ├── sentry.yaml            哨兵：hikrobot · 曝光 0.8ms · gain 16.9
│   └── uav.yaml               无人机：⭐ **mindvision** · 曝光 8ms · gamma 0.6
│
└── (预生成的弹道表等)
```

⚠️ **W102 之前的错误分层**（已纠正，留作教训）：
```
params/cameras/
  hikrobot.yaml    ✅ 品牌
  infantry.yaml    ❌ 兵种级的也塞这儿了 ⇒ 和 robots/ 职责重叠
```

---

## ⭐ 加载顺序（**只有两层**）

```
① params/cameras/<品牌>.yaml          ← 品牌默认（最低）
② params/robots/<兵种>.yaml           ← 兵种覆盖（最高，也是唯一要改的地方）
```

⭐ **品牌文件怎么找到的**：由兵种 yaml 的 `camera_name` 推出
`params/cameras/<camera_name>.yaml`。
⚠️ **跟兵种 yaml 的位置走，不依赖当前工作目录**：
```
params/robots/infantry.yaml  →  params/cameras/hikrobot.yaml
params/robots/uav.yaml       →  params/cameras/mindvision.yaml
```

⭐ **`--camera-config=<path>`** 可**覆盖**品牌文件（临时试一组品牌级参数时用）。

---

## ⭐ 先看当前生效值（**别猜**）

```bash
tools/scripts/run.sh infantry --print-config params/robots/infantry.yaml
```
`--print-config` 的 **③ 相机** 节会打出**两层合并后**的
`camera_name` / `vid_pid` / `exposure_ms` / `gain` / `fps`。

---

## ⭐ 三层配置的"谁覆盖谁"（完整图）

```
① 命令行参数（CLI）          ← 最高（--tongji / --camera-config / --bullet-speed …）
② params/robots/<兵种>.yaml  ← 兵种配置（含相机段）
③ params/cameras/<品牌>.yaml ← 品牌默认
④ 源码内置默认               ← 最低
```

⚠️ **调试"为什么是这个值"** ⇒ 一律先跑 `--print-config`，它打的是**最终生效值**。

---

## ⭐ 相机参数怎么写

### 兵种自己改（最常用）
```yaml
# params/robots/infantry.yaml
#####----- ⭐⭐⭐ 相机（本兵种特有）-----#####
camera_name: "hikrobot"      # ⭐ 品牌 → 自动带出 params/cameras/hikrobot.yaml
vid_pid: "2bdf:0001"         # 插了多台时按它选
exposure_ms: 10              # 曝光（ms）—— ⚠️ 画面暗先加这个
gain: 16                     # 增益（dB）
fps: 30
```

### 品牌通用参数（换海康相机可能要改）
```yaml
# params/cameras/hikrobot.yaml
camera_params:
  enum:
    BalanceWhiteAuto: 0      # ⭐ 0=Off（默认 2 会漂）
    # PixelFormat: 17301512  # BayerRG8（不是 Bayer8 会报错，见下）
  float:
    # Gamma: 1.0
```

⭐ **不知道填多少** ⇒ 先在 MVS 里调好，再 `--dump-camera-params` 打出来抄进去。

---

## ⚠️ 常见坑

| 现象 | 原因 / 怎么办 |
|---|---|
| **画面暗** | `exposure_ms` 太小 ⇒ 先加它（`gain` 会放大噪声） |
| **`MVS 看着亮、程序采的暗`** | ⚠️ **程序会强制覆盖** `ExposureTime`/`Gain`（`ExposureAuto=OFF`）⇒ **以我们 yaml 为准**，不是 MVS 里看到的 |
| **报「不支持的像素格式」** | 相机输出的不是 4 种 8 位 Bayer ⇒ 在**品牌文件**里指定 `PixelFormat`（W93 起**不崩**，只跳帧 + 明确报错） |
| **插了两台海康，选错** | 设 `vid_pid`（日志会打「选中设备 #1/2：…」） |
| **颜色红蓝反了** | `PixelFormat` 的 Bayer 排列不对（`RG/GR/GB/BG`） |

⭐ 单帧诊断工具：`python3 tools/diagnose_frame.py <图>`（会和 demo 帧并排对比）
