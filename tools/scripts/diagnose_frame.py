#!/usr/bin/env python3
"""
⭐ 单帧诊断 —— 对比【真实相机帧】与【录像帧】，定位"识别不出"的原因

用法:
    # ① 先抓真实相机的图（按热键 4，或跑 --debug-img=true）
    tools/scripts/run.sh infantry --debug-img=true        # 每 30 帧存 1 张
    # ② 拿存下来的图诊断
    python3 tools/scripts/diagnose_frame.py output/images/000030_aim.png

它会打印：分辨率 / 亮度 / 对比度 / BGR 通道均值 / 与录像帧的差异，
并按 YOLO 的**同一套预处理**（resize 到左上 + 黑边）算一遍，帮你判断是
"图太暗 / 太糊 / 颜色不对 / 确实没装甲板"。
"""
import sys, os, glob
import cv2
import numpy as np

def stats(img, label):
    b, g, r = img[:, :, 0].mean(), img[:, :, 1].mean(), img[:, :, 2].mean()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 清晰度：Laplacian 方差（越小越糊）
    lap = cv2.Laplacian(gray, cv2.CV_64F).var()
    print(f"  {label}")
    print(f"    尺寸      {img.shape[1]}x{img.shape[0]}")
    # ⭐ 阈值按【实测能识别装甲板的帧】校准（demo.avi 检出 2-3 个的帧）：
    #     亮度 13.9~21.0 · 对比度 14.7~22.4 · Laplacian 24.1~33.7 · R-B -7~-18
    #   ⚠️ 这个数据集本身很暗、对比度很低，所以阈值不能按"正常照片"定
    print(f"    亮度      均值 {gray.mean():5.1f}   ⭐ 可识别区间 14~21"
          f"   {'✅' if 8 <= gray.mean() <= 60 else '⚠️ 偏离'}")
    print(f"    对比度    std {gray.std():5.1f}   ⭐ 可识别区间 15~22"
          f"   {'✅' if gray.std() >= 12 else '⚠️ 太平（糊/无纹理）'}")
    print(f"    清晰度    Laplacian 方差 {lap:8.1f}   ⭐ 可识别区间 24~34"
          f"   {'✅' if lap >= 15 else '⚠️ 很糊（对焦？）'}")
    print(f"    BGR 均值  [{b:5.1f}, {g:5.1f}, {r:5.1f}]"
          f"   {'⚠️ 偏蓝' if b > r * 1.4 else ('⚠️ 偏红' if r > b * 1.4 else '✅ 中性')}")
    print(f"    R-B 差    {r - b:+6.1f}   ← ⭐ 负=偏蓝 正=偏红")
    return dict(mean=gray.mean(), std=gray.std(), lap=lap, b=b, g=g, r=r)

def yolo_preprocess(img):
    """⭐ 复刻 yolov5.cpp 的预处理：保持长宽比缩到左上角，右下补黑"""
    x_scale = 640.0 / img.shape[0]
    y_scale = 640.0 / img.shape[1]
    s = min(x_scale, y_scale)
    h, w = int(img.shape[0] * s), int(img.shape[1] * s)
    canvas = np.zeros((640, 640, 3), np.uint8)
    canvas[:h, :w] = cv2.resize(img, (w, h))
    return canvas, w, h

def main():
    if len(sys.argv) < 2:
        print(__doc__); return 1
    path = sys.argv[1]
    img = cv2.imread(path)
    if img is None:
        print(f"  ❌ 读不到 {path}"); return 1
    print(f"  ════ 诊断 {os.path.basename(path)} ════\n")
    s1 = stats(img, "原始帧")

    # ⭐ 预处理后的黑边占比
    _, w, h = yolo_preprocess(img)
    pad = (640 * 640 - w * h) / (640 * 640) * 100
    print(f"\n    YOLO 预处理: 内容 {w}x{h} 塞进 640x640，"
          f"⭐ 黑边占 {pad:.0f}%   （视频帧约 25%，相机帧约 44%）")

    # ⭐ 与录像帧对比（如果存在）
    demo = 'assets/demo/demo.avi'
    if os.path.exists(demo):
        cap = cv2.VideoCapture(demo)
        ok, f = cap.read(); cap.release()
        if ok:
            print()
            s2 = stats(f, "对照：demo.avi 第 1 帧（识别得出的那种）")
            print(f"\n  ════ 差异 ════")
            print(f"    亮度   {s1['mean']-s2['mean']:+7.1f}   对比度 {s1['std']-s2['std']:+7.1f}")
            print(f"    清晰度 {s1['lap']-s2['lap']:+9.1f}   R-B    {s1['r']-s1['b']:+7.1f}"
                  f"  (demo: {s2['r']-s2['b']:+.1f})")
            print()
            print("    ⭐ 逐项判断（阈值来自 demo.avi 能识别的帧）：")
            hit = False
            if s1['lap'] < 15:
                print("      ⚠️ 清晰度远低于可识别区间 → **偏糊**：查对焦/距离"); hit = True
            if s1['mean'] < 8:
                print("      ⚠️ 太暗 → 调大 `exposure_ms`（当前 2）或 `gain`"); hit = True
            if s1['mean'] > 60:
                print("      ⚠️ 偏亮/过曝 → 调小 `exposure_ms`"); hit = True
            if s1['std'] < 12:
                print("      ⚠️ 对比度极低 → 画面没细节（糊/全黑/全白）"); hit = True
            if not hit:
                print("      ✅ 各项都在可识别区间内")
                print("      ⇒ 那更可能是【Bayer 转换 / 色彩】或【画面里真没有装甲板】")
                print("         （objectness 0.002 = 模型认为完全不像装甲板，")
                print("           而 demo 能识别时 objectness 是 0.1~0.27，差 50 倍以上）")
    return 0

if __name__ == '__main__':
    sys.exit(main())
