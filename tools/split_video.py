#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
split_video.py —— ⭐ 切分录像（`.avi` + 同名 `.txt` 位姿）的帧区间

## 来源
同济 `calibration/split_video.cpp`（C++）。本项目**改写为 Python**，理由：
1. 它是**纯文件操作**（不碰硬件/算法），放 `tools/scripts/` 更合适
2. C++ 版要重编；Python 版改区间**不用编译**，调参更快
3. ⭐ 零额外依赖（只用 `cv2`，本机已有）

## 用法
```bash
# 取第 100..300 帧，另存为 records/clip.avi + clips/clip.txt
python3 tools/split_video.py records/full.avi -s 100 -e 300 -o records/clip

# 只切位姿文本（不动视频），看时间戳
python3 tools/split_video.py records/full.avi -s 100 -e 300 --txt-only
```
"""
import argparse
import os
import sys

try:
    import cv2
except ImportError:
    print("需要 opencv-python（cv2）"); sys.exit(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="输入 .avi 路径（同名 .txt 是位姿）")
    ap.add_argument("-s", "--start-index", type=int, default=0)
    ap.add_argument("-e", "--end-index", type=int, default=-1, help="-1 = 到结尾")
    ap.add_argument("-o", "--output-path", default=None, help="输出前缀（不含扩展名）")
    ap.add_argument("--txt-only", action="store_true", help="只切位姿文本，不写视频")
    ap.add_argument("--fps", type=float, default=None, help="输出帧率（默认沿用输入）")
    a = ap.parse_args()

    base = os.path.splitext(a.input)[0]
    txt_path = base + ".txt"
    out = a.output_path or (base + "_clip")

    # ── 位姿文本 ──
    lines = []
    if os.path.exists(txt_path):
        with open(txt_path) as f:
            lines = [ln.rstrip("\n") for ln in f if ln.strip()]
        print(f"位姿: {txt_path}  共 {len(lines)} 行")
    else:
        print(f"⚠️  没有位姿文件 {txt_path}（只切视频）")

    end = a.end_index if a.end_index >= 0 else (len(lines) if lines else 10 ** 9)

    if lines:
        os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
        with open(out + ".txt", "w") as f:
            for ln in lines[a.start_index:end]:
                f.write(ln + "\n")
        print(f"✅ 位姿 -> {out}.txt  （{max(0, min(end, len(lines)) - a.start_index)} 行）")

    if a.txt_only:
        return 0

    # ── 视频 ──
    cap = cv2.VideoCapture(a.input)
    if not cap.isOpened():
        print(f"❌ 打不开视频 {a.input}"); return 1
    fps = a.fps or cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"视频: {a.input}  {w}x{h} @ {fps:.2f}fps  共 {total} 帧")

    os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
    vw = cv2.VideoWriter(out + ".avi", cv2.VideoWriter_fourcc(*"MJPG"), fps, (w, h))
    cap.set(cv2.CAP_PROP_POS_FRAMES, a.start_index)
    written = 0
    for i in range(a.start_index, min(end, total) if total > 0 else end):
        ok, frame = cap.read()
        if not ok:
            break
        vw.write(frame)
        written += 1
    vw.release(); cap.release()
    print(f"✅ 视频 -> {out}.avi  （{written} 帧）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
