# -*- coding: utf-8 -*-
"""
00 · 制作 CharUco 标定板（可打印，物理尺寸精确）。

输出（默认 p2.Calibrate/board/）：
  board.png        高分辨率位图（已嵌 DPI，打印选“实际尺寸/100%”即可）
  board.pdf        ★推荐打印此文件：板按精确毫米铺版，方格严格等于设定边长
  board_spec.json  规格（方格/标记边长、行列、字典、像素/方格、DPI）

原理：OpenCV CharucoBoard.generateImage(outSize, marginSize=0) 在 outSize=(sx*K, sy*K)
时每格恰为 K 像素（已实测）。据此把图按 “每格=设定毫米” 铺进 PDF，打印即得精确尺寸。

用法（默认取 config.py）:
  conda run -n rm65 python scripts/00_make_board.py
自定义（不改 config）:
  conda run -n rm65 python scripts/00_make_board.py --square-mm 25 --marker-mm 18 --sx 8 --sy 6
"""
import os
import sys
import json
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import cv2.aruco as aruco
import numpy as np

import config as C
from charuco import detect


def build_board(sx, sy, square_m, marker_m, dict_name):
    if dict_name not in C.ARUCO_DICTS:
        raise ValueError(f"未知字典 {dict_name}，可选 {list(C.ARUCO_DICTS)}")
    dictionary = aruco.getPredefinedDictionary(C.ARUCO_DICTS[dict_name])
    return aruco.CharucoBoard((sx, sy), square_m, marker_m, dictionary)


def run(args):
    sx, sy = args.sx, args.sy
    square_mm = args.square_mm
    marker_mm = args.marker_mm
    K = args.ppi          # 像素/方格（控制打印清晰度）
    square_m = square_mm / 1000.0
    marker_m = marker_mm / 1000.0

    board = build_board(sx, sy, square_m, marker_m, args.dict)

    # 1) 高分辨率渲染，marginSize=0 ⇒ 每格恰为 K px
    img = board.generateImage((sx * K, sy * K), marginSize=0, borderBits=1)

    # 2) 校验每格像素（用检测到的棋盘角点列间距）
    corners, ids = detect(aruco.CharucoDetector(board), img)
    if corners is None:
        sys.exit("[!] 渲染后自检失败，无法确认每格像素；尝试更大 --ppi")
    xs = np.sort(np.unique(np.round(corners[:, 0]).astype(int)))
    pitch = int(round(np.median(np.diff(xs)[np.diff(xs) > 1])))
    if abs(pitch - K) > 1:
        sys.exit(f"[!] 每格像素 {pitch} != 期望 {K}，几何假设不成立，需排查")
    print(f"[board] 每格像素校验通过：{pitch}px/格")

    # 3) 物理尺寸（mm）
    w_mm = sx * square_mm
    h_mm = sy * square_mm
    dpi = K / square_mm * 25.4     # 使每格在打印时 = square_mm
    print(f"[board] 物理尺寸 {w_mm:.1f}×{h_mm:.1f} mm，{sx}×{sy} 格，"
          f"方格{square_mm}mm/标记{marker_mm}mm，字典{args.dict}，DPI≈{dpi:.0f}")

    # 4) 输出目录
    out_dir = C.ROOT / (args.out_dir or "board")
    out_dir.mkdir(parents=True, exist_ok=True)
    png_path = out_dir / "board.png"
    pdf_path = out_dir / "board.pdf"
    spec_path = out_dir / "board_spec.json"

    cv2.imwrite(str(png_path), img)

    # 嵌入 DPI，便于直接打印 PNG（仍推荐用 PDF）
    try:
        from PIL import Image
        im = Image.open(png_path)
        im.save(png_path, dpi=(dpi, dpi))
        print(f"[board] PNG 已嵌 DPI={dpi:.0f}")
    except Exception as e:
        print(f"[board] PNG 嵌 DPI 跳过（{e}）——请打印 PDF")

    # 5) PDF：按精确毫米铺版
    margin = args.margin_mm
    caption_h = 14
    page_w = w_mm + 2 * margin
    page_h = h_mm + 2 * margin + caption_h
    from fpdf import FPDF
    # 注意：fpdf2 对 orientation='L' 会强制交换宽高(w=fh,h=fw)，
    # 故这里用默认 'P' + format=(page_w,page_h)，保证页面宽=page_w、高=page_h，板不被裁切。
    pdf = FPDF(unit="mm", format=(page_w, page_h))
    pdf.set_auto_page_break(False)
    pdf.add_page()
    pdf.image(str(png_path), x=margin, y=margin, w=w_mm, h=h_mm)
    # 说明条（板外，不影响检测）
    pdf.set_xy(margin, margin + h_mm + 3)
    pdf.set_font("Helvetica", size=9)
    cap = (f"CharUco {sx}x{sy}  square={square_mm}mm  marker={marker_mm}mm  dict={args.dict}  "
           f"|  print at ACTUAL SIZE/100%, verify each square={square_mm}mm with caliper")
    pdf.cell(0, 6, text=cap)
    pdf.output(str(pdf_path))
    print(f"[board] PDF 页面 {page_w:.1f}x{page_h:.1f} mm，板精确 {w_mm:.1f}x{h_mm:.1f} mm")

    # 6) 规格留档
    spec = {
        "type": "charuco", "dict": args.dict,
        "squares_x": sx, "squares_y": sy,
        "square_len_mm": square_mm, "marker_len_mm": marker_mm,
        "physical_mm": [w_mm, h_mm], "px_per_square": K, "dpi": dpi,
        "files": {"png": "board.png", "pdf": "board.pdf"},
    }
    spec_path.write_text(json.dumps(spec, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"\n[done] 输出目录 {out_dir}")
    print("  打印：用 board.pdf，打印对话框选“实际尺寸/100%”，不要缩放；")
    print(f"  印后用卡尺复核每格应为 {square_mm}mm；裱在硬板/铝板上保持平整。")
    if square_mm != round(C.BOARD_SQUARE_LEN_M * 1000) or marker_mm != round(C.BOARD_MARKER_LEN_M * 1000) \
       or sx != C.BOARD_SQUARES_X or sy != C.BOARD_SQUARES_Y or args.dict != C.ARUCO_DICT:
        print("  ⚠️ 本次参数与 config.py 不同：务必同步修改 config.py 的板参数，否则 solvePnP 会错！")


def main():
    ap = argparse.ArgumentParser(description="生成可打印的 CharUco 标定板")
    ap.add_argument("--sx", type=int, default=C.BOARD_SQUARES_X)
    ap.add_argument("--sy", type=int, default=C.BOARD_SQUARES_Y)
    ap.add_argument("--square-mm", type=float, default=C.BOARD_SQUARE_LEN_M * 1000)
    ap.add_argument("--marker-mm", type=float, default=C.BOARD_MARKER_LEN_M * 1000)
    ap.add_argument("--dict", default=C.ARUCO_DICT)
    ap.add_argument("--ppi", type=int, default=600, help="每方格像素（渲染清晰度，默认 600≈508DPI）")
    ap.add_argument("--margin-mm", type=float, default=8.0, help="PDF 页边距(mm)")
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args()
    if args.marker_mm >= args.square_mm:
        sys.exit("[!] 标记边长须小于方格边长")
    run(args)


if __name__ == "__main__":
    main()
