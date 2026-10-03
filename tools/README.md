# `tools/` —— ⭐ **调试工具**（不限语言）

> ⭐ **判据**：**"我为了『看 / 查 / 诊断』而运行的东西"** ⇒ 放这里。
> ⚠️ **不限语言，也不要求"跟项目无关"** —— 认识本项目数据的工具**也**放这里
> （例如 `csv_viewer.html` 就认识我们的 CSV 字段，但它显然是个**工具**）。

## 三个目录的分工（⭐ 别放错）

| 目录 | 放什么 | 语言 | 判据 |
|---|---|---|---|
| ⭐ **`tools/`**（本目录） | **调试工具** | **不限**（py / html / …） | 我为了**看/查/诊断**而跑的 |
| **`tools/scripts/`** | **构建 / 运行 / 硬件脚本** | ⭐ **只放 sh** | 我需要**执行一组命令**时用的 |
| **`scripts/`** | ⭐ **跟项目业务强相关的 py** | ⭐ **只放 py** | 它处理**项目的数据/算法/指标** |
| **`imgs/`** | 文档配图 | png/svg | 被 `.md` 引用的图 |

⚠️ **曾经的错位（已修，W101）**：
- `tools/scripts/diagnose_frame.py` —— ⚠️ **Python 跑在 sh 目录里** ⇒ 移到 `tools/`
- `scripts/example_perf.png` —— ⚠️ **图片跑在 py 目录里** ⇒ 移到 `imgs/`
- `scripts/__pycache__` —— ⚠️ 编译产物（`.gitignore` 已规则，只是磁盘残留）⇒ 清掉

---

## 本目录的文件

| 文件 | 干什么 | 依赖 |
|---|---|---|
| `csv_viewer.html` | ⭐ **单文件** CSV 查看器：**浏览器直接拖 CSV 进去**（零安装、零依赖） | 浏览器 |
| `diagnose_frame.py` | ⭐ **单帧图像诊断**：亮度/对比度/Laplacian/R−B + **与 demo 帧并排对比** | `opencv-python` `numpy` |
| `split_video.py` | ⭐ **切录像**：按帧区间切成 `clip.avi` + `clip.txt`（做 A/B 实验用） | `opencv-python` |

---

## 用法

### `csv_viewer.html` —— 看 CSV（最常用）
```
直接把 *_frames.csv 拖进浏览器
```
⭐ 为什么是 HTML：**赛场上不一定装 pandas / matplotlib**，而浏览器一定有。
⭐ 它**认识我们的字段**（`det_armor_count` / `pln_t_plan_us` / `trk_*` …）⇒ 自动分组、画图。

### `diagnose_frame.py` —— 单帧为什么识别不到
```bash
# ① 先抓图（真机按键 4，或跑 --debug-img=true 每 30 帧存 1 张）
tools/scripts/run.sh infantry --debug-img=true
# ② 诊断
python3 tools/diagnose_frame.py output/images/000030_aim.png
```
⭐ 它会把**你的帧**和 **`assets/demo/demo.avi` 第 1 帧**（"能识别出来的那种"）**并排对比**，
**逐项打勾/打叉**。⚠️ 阈值是**按 demo 校准的**（那个数据集本身很暗），**别拿"正常照片"的标准看**。

### `split_video.py` —— 切一小段做 A/B 实验
```bash
python3 tools/split_video.py records/full.avi -s 100 -e 300 -o records/clip
# ⇒ records/clip.avi + records/clip.txt（位姿）。之后：
#    tools/scripts/run.sh infantry --video=records/clip.avi --force-mode=1
```

---

## ⭐ 什么**不该**放这里

| ❌ 别放 | 该去哪 | 为什么 |
|---|---|---|
| `build.sh` / `run.sh` 这类 | `tools/scripts/` | 它是 **sh 脚本** |
| 业务算法验证 / 指标分析 | `scripts/` | 它处理**项目的数据/算法** |
| 图片 | `imgs/` | 被文档引用 |
