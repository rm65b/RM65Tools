# -*- coding: utf-8 -*-
"""
02 · 内参复校（可选）。

用采集到的样本（或专门的棋盘图）重新标定相机内参 K、畸变 dist。
D435 出厂内参对手眼足够，仅当对精度有更高要求时运行。

用法:
  python scripts/02_calibrate_intrinsics.py --serial 261722075459
"""
import os
import sys
import json
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np

import config as C
from charuco import make_board, detect, board_objpoints
from realsense_cam import save_intrinsics


def run(args):
    serial = args.serial
    mf = C.samples_dir(serial) / "samples.json"
    if not mf.exists():
        sys.exit(f"[!] 找不到样本清单 {mf}，先运行 01_collect.py")
    manifest = json.loads(mf.read_text(encoding="utf-8"))

    board, detector = make_board()
    obj_pts_list, img_pts_list, im_size = [], [], None
    for s in manifest["samples"]:
        img = cv2.imread(str(C.ROOT / s["image"]))
        if img is None:
            continue
        if im_size is None:
            im_size = img.shape[:2][::-1]   # (w,h)
        corners, ids = detect(detector, img)
        if corners is None:
            continue
        obj_pts_list.append(board_objpoints(board, ids).reshape(-1, 1, 3).astype(np.float64))
        img_pts_list.append(corners.reshape(-1, 1, 2).astype(np.float64))

    if len(obj_pts_list) < 8:
        sys.exit(f"[!] 可用图像仅 {len(obj_pts_list)} 张(<8)，建议补采后复校。")

    print(f"[intrinsics] 用 {len(obj_pts_list)} 张图复校 {serial} ...")
    flags = 0
    ret, K, dist, _rvecs, _tvecs = cv2.calibrateCamera(
        obj_pts_list, img_pts_list, im_size, None, None, flags=flags)
    dist = dist.reshape(1, -1)
    print(f"  重投影误差 RMS = {ret:.4f} px")
    print(f"  K =\n{np.round(K,2)}")
    print(f"  dist = {np.round(dist.reshape(-1),4).tolist()}")

    intr = {
        "K": K, "dist": dist,
        "width": im_size[0], "height": im_size[1],
        "fx": K[0, 0], "fy": K[1, 1], "ppx": K[0, 2], "ppy": K[1, 2],
        "model": "recalibrated",
    }
    p = save_intrinsics(serial, intr, source="recalibrated")
    print(f"[done] 已写入 {p}  (source=recalibrated)")
    if ret > 1.0:
        print("[!] 重投影误差 >1px，检查标定板物理尺寸配置 / 图像清晰度。")


def main():
    ap = argparse.ArgumentParser(description="相机内参复校")
    ap.add_argument("--serial", required=True)
    args = ap.parse_args()
    run(args)


if __name__ == "__main__":
    main()
