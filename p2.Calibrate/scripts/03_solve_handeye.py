# -*- coding: utf-8 -*-
"""
03 · 手眼 / 眼在手外 解算。

输入：01 采集的 samples.json（图像+法兰位姿）+ intrinsics.json
输出：handeye.json（眼在手上存 T_flange_cam）或 eye_to_hand.json（眼在手外存 T_base_cam）

方法：
  - eye_in_hand : AX=XB, X = T_flange_cam，直接用 cv2.calibrateHandEye
  - eye_to_hand : 把机器人位姿取逆后等价为 eye_in_hand，输出 X = T_base_cam
  四种方法(TSAI/PARK/DANIILIDIS/ANDREFF)各解一遍，按“链一致性残差”挑最优。

一致性度量（与 cv2 内部 A,B 无关，直接验物理常量）：
  - eye_in_hand : 标定板世界固定 → T_BTg(k)=T_BF(k)@X@T_ATg(k) 应恒定
  - eye_to_hand : 标定板固连末端 → T_F_Tg(k)=inv(T_BF(k))@X@T_ETg(k) 应恒定
  取其跨样本 RMS 离散度（旋转°/平移mm）。

用法:
  python scripts/03_solve_handeye.py --serial 261722075459
  python scripts/03_solve_handeye.py --serial 261722076078
"""
import os
import sys
import json
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np

import config as C
from charuco import make_board, detect, estimate_pose, board_objpoints
from realsense_cam import load_intrinsics
from transforms import pose_to_mat4, invert, logmap_rot, chain_consistency

METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
}

# 验收阈值
ACCEPT_ROT_DEG = 1.0
ACCEPT_TRANS_MM = 3.0
ACCEPT_REPROJ_PX = 1.0


def reproj_px(K, dist, objpts, imgpts, T_cam_board):
    rvec, _ = cv2.Rodrigues(T_cam_board[:3, :3])
    tvec = T_cam_board[:3, 3].reshape(3, 1)
    proj, _ = cv2.projectPoints(objpts.reshape(-1, 1, 3), rvec, tvec, K, dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - imgpts.reshape(-1, 2), axis=1)
    return float(err.mean()), float(err.max())


def load_dataset(serial):
    mf = C.samples_dir(serial) / "samples.json"
    if not mf.exists():
        sys.exit(f"[!] 找不到样本清单 {mf}")
    manifest = json.loads(mf.read_text(encoding="utf-8"))
    if not C.intrinsics_path(serial).exists():
        sys.exit(f"[!] 找不到内参 {C.intrinsics_path(serial)}（先跑 01 或 02）")
    intr = load_intrinsics(serial)
    board, detector = make_board()
    K, dist = intr["K"], intr["dist"]
    unit = manifest.get("pose_unit", C.POSE_RPY_UNIT)
    order = manifest.get("pose_order", C.POSE_RPY_ORDER)

    T_BF, T_CT, reproj = [], [], []
    for s in manifest["samples"]:
        img = cv2.imread(str(C.ROOT / s["image"]))
        if img is None:
            print(f"  [!] 读图失败 {s['image']}，跳过")
            continue
        corners, ids = detect(detector, img)
        if corners is None:
            print(f"  [!] 检测失败 #{s['idx']}，跳过")
            continue
        T_bf = pose_to_mat4(s["pose"], unit=unit, order=order)
        T_ct = estimate_pose(board, corners, ids, intr)
        obj = board_objpoints(board, ids)
        m, _mx = reproj_px(K, dist, obj, corners, T_ct)
        T_BF.append(T_bf); T_CT.append(T_ct); reproj.append(m)
    return manifest, intr, T_BF, T_CT, np.array(reproj)


def solve(manifest, T_BF, T_CT):
    mode = manifest["mode"]
    n = len(T_BF)
    R_g2b = [T[:3, :3] for T in T_BF]
    t_g2b = [T[:3, 3] for T in T_BF]
    R_t2c = [T[:3, :3] for T in T_CT]
    t_t2c = [T[:3, 3] for T in T_CT]

    per_method = {}
    for name, flag in METHODS.items():
        if mode == "eye_in_hand":
            R_x, t_x = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=flag)
            X = np.eye(4); X[:3, :3] = R_x; X[:3, 3] = t_x.ravel()
            chain = [T_BF[k] @ X @ T_CT[k] for k in range(n)]
        else:   # eye_to_hand：机器人位姿取逆
            R_b2g = [R.T for R in R_g2b]
            t_b2g = [-(R_g2b[k].T @ t_g2b[k]) for k in range(n)]
            R_x, t_x = cv2.calibrateHandEye(R_b2g, t_b2g, R_t2c, t_t2c, method=flag)
            X = np.eye(4); X[:3, :3] = R_x; X[:3, 3] = t_x.ravel()
            chain = [invert(T_BF[k]) @ X @ T_CT[k] for k in range(n)]
        rot_rms, trans_rms = chain_consistency(chain)
        per_method[name] = {"X": X, "rot_deg": rot_rms, "trans_mm": trans_rms}
        print(f"  {name:11s}  rot={rot_rms:.3f}°  trans={trans_rms:.2f}mm")

    best = min(per_method, key=lambda k: (per_method[k]["trans_mm"], per_method[k]["rot_deg"]))
    return best, per_method


def run(args):
    serial = args.serial
    manifest, intr, T_BF, T_CT, reproj = load_dataset(serial)
    if len(T_BF) < C.AX_XB_MIN_PAIRS:
        print(f"[!] 仅 {len(T_BF)} 个有效样本(<{C.AX_XB_MIN_PAIRS})，结果可能不可靠。")
    print(f"[solve] {serial}  模式={manifest['mode']}  有效样本={len(T_BF)}")
    print(f"[solve] solvePnP 平均重投影 = {reproj.mean():.3f}px (max {reproj.max():.3f}px)")

    best, per_method = solve(manifest, T_BF, T_CT)
    X = per_method[best]["X"]
    rot = per_method[best]["rot_deg"]
    trans = per_method[best]["trans_mm"]
    print(f"\n[best] 方法={best}  X ({'T_flange_cam' if manifest['mode']=='eye_in_hand' else 'T_base_cam'}):")
    print(np.round(X, 5))
    print(f"  一致性残差: rot={rot:.3f}°  trans={trans:.2f}mm")

    out = {
        "serial": serial,
        "type": manifest["mode"],   # eye_in_hand | eye_to_hand
        "transform": X.tolist(),
        "method": best,
        "num_poses": len(T_BF),
        "residual_rot_deg": rot,
        "residual_trans_mm": trans,
        "reproj_mean_px": float(reproj.mean()),
        "intrinsic_source": intr.get("source", "factory"),
        "pose_unit": manifest.get("pose_unit", C.POSE_RPY_UNIT),
        "pose_order": manifest.get("pose_order", C.POSE_RPY_ORDER),
        "flange_d6_mm": C.FLANGE_D6_MM,
        "methods": {k: {"rot_deg": v["rot_deg"], "trans_mm": v["trans_mm"],
                        "transform": v["X"].tolist()} for k, v in per_method.items()},
        "created_at": "2026-06-23",
    }
    C.handeye_path(serial).parent.mkdir(parents=True, exist_ok=True)
    C.handeye_path(serial).write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n[done] 已写入 {C.handeye_path(serial)}")

    ok_rot = rot <= ACCEPT_ROT_DEG
    ok_trans = trans <= ACCEPT_TRANS_MM
    ok_reproj = reproj.mean() <= ACCEPT_REPROJ_PX
    print("\n=== 验收 ===")
    print(f"  旋转一致性 {rot:.3f}°  {'OK' if ok_rot else 'NG'} (≤{ACCEPT_ROT_DEG}°)")
    print(f"  平移一致性 {trans:.2f}mm {'OK' if ok_trans else 'NG'} (≤{ACCEPT_TRANS_MM}mm)")
    print(f"  重投影     {reproj.mean():.3f}px {'OK' if ok_reproj else 'NG'} (≤{ACCEPT_REPROJ_PX}px)")
    if not (ok_rot and ok_trans and ok_reproj):
        print("  [!] 未达标：见 标定方案.md §8 排查（优先查位姿约定/单位与奇异点）。")


def main():
    ap = argparse.ArgumentParser(description="手眼/眼在手外 解算")
    ap.add_argument("--serial", required=True)
    args = ap.parse_args()
    run(args)


if __name__ == "__main__":
    main()
