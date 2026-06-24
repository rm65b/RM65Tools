# -*- coding: utf-8 -*-
"""
04 · 验证。

离线（默认）：对某相机复核 solvePnP 重投影、链一致性残差、变换矩阵合法性，
            并打印可直接使用的映射公式。
实时跨相机（--live）：放一块 CharUco 让两路相机同时看见，各自映射到基座系，
            比对同一点坐标差（最能暴露坐标系/符号错误）。

用法:
  python scripts/04_verify.py --serial 261722075459          # 离线复核
  python scripts/04_verify.py --live                         # 实时跨相机比对(需两路已标定+机械臂)
"""
import os
import sys
import json
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np

import config as C
from charuco import make_board, detect, estimate_pose
from realsense_cam import load_intrinsics, RealSenseCam
from transforms import pose_to_mat4, invert, is_orthonormal, logmap_rot


def load_result(serial):
    p = C.handeye_path(serial)
    if not p.exists():
        return None
    return json.loads(p.read_text(encoding="utf-8"))


def verify_offline(serial):
    res = load_result(serial)
    if res is None:
        sys.exit(f"[!] {serial} 未解算，先运行 03_solve_handeye.py")
    X = np.array(res["transform"])
    print(f"=== 离线复核 {serial} ({res['type']}) ===")
    print(f"  方法={res['method']}  样本数={res['num_poses']}")
    print(f"  一致性残差 rot={res['residual_rot_deg']:.3f}°  trans={res['residual_trans_mm']:.2f}mm")
    print(f"  重投影={res['reproj_mean_px']:.3f}px")
    print(f"  X 正交良好: {is_orthonormal(X[:3,:3])}")
    if res["type"] == "eye_in_hand":
        print("\n  使用: P_base = T_base_flange(now) @ X @ P_camA ,  X=T_flange_cam =")
    else:
        print("\n  使用: P_base = X @ P_camB ,  X=T_base_cam =")
    print(np.round(X, 5))
    print("\n  其逆（如需相机系反查）:")
    print(np.round(invert(X), 5))


def verify_live():
    """两路同框：放一块静态 CharUco，两路同时看到，映射到基座系比对。"""
    rA = load_result(C.SERIAL_EYE_IN_HAND)
    rB = load_result(C.SERIAL_EYE_TO_HAND)
    if rA is None or rB is None:
        sys.exit("[!] 两路都需先解算（03）。")
    T_F_A = np.array(rA["transform"])     # 眼在手上：相机A→法兰
    T_B_E = np.array(rB["transform"])     # 眼在手外：相机B→基座
    unit = rA.get("pose_unit", C.POSE_RPY_UNIT)
    order = rA.get("pose_order", C.POSE_RPY_ORDER)

    from arm_rmm import make_arm
    arm = make_arm("sdk", ip=None, port=None)
    camA = RealSenseCam(C.SERIAL_EYE_IN_HAND).start()
    camB = RealSenseCam(C.SERIAL_EYE_TO_HAND).start()
    intrA = load_intrinsics(C.SERIAL_EYE_IN_HAND)
    intrB = load_intrinsics(C.SERIAL_EYE_TO_HAND)
    board, detector = make_board()

    print("\n=== 实时跨相机比对 ===")
    print("放一块静态 CharUco，使两路同时可见；摆好臂位姿后回车测量（q 退出）。")
    while True:
        cmd = input("回车=测量  q=退出: ").strip().lower()
        if cmd == "q":
            break
        _joints, pose = arm.get_state()
        T_BF = pose_to_mat4(pose, unit=unit, order=order)

        cA, _d, _ = camA.grab()
        cB, _d, _ = camB.grab()
        ca, ia = detect(detector, cA)
        cb, ib = detect(detector, cB)
        if ca is None or cb is None:
            print("  [x] 两路未同时检测到板，调整位置。")
            continue
        T_A_Tg = estimate_pose(board, ca, ia, intrA)   # 板在A系
        T_E_Tg = estimate_pose(board, cb, ib, intrB)   # 板在B系
        p_board_A = T_A_Tg[:3, 3]
        p_board_E = T_E_Tg[:3, 3]
        # 映射到基座系
        pA_base = T_BF @ (T_F_A @ np.append(p_board_A, 1.0))
        pB_base = T_B_E @ np.append(p_board_E, 1.0)
        delta = pA_base[:3] - pB_base[:3]
        print(f"  经A(眼在手上) → 基座: {np.round(pA_base[:3],2)} (mm: {np.round(pA_base[:3]*1000,1)})")
        print(f"  经B(眼在手外) → 基座: {np.round(pB_base[:3],2)} (mm: {np.round(pB_base[:3]*1000,1)})")
        print(f"  差值 Δ = {np.round(delta,4)*1000} mm  |Δ|={np.linalg.norm(delta)*1000:.2f} mm")
        if np.linalg.norm(delta) * 1000 < 5.0:
            print("  -> 两路一致(<5mm)，标定可靠。")
        else:
            print("  -> 差值偏大，查坐标系约定/符号/位姿单位。")
    camA.stop(); camB.stop(); arm.close()


def main():
    ap = argparse.ArgumentParser(description="标定结果验证")
    ap.add_argument("--serial", default=None, help="离线复核指定相机")
    ap.add_argument("--live", action="store_true", help="实时两路同框比对")
    args = ap.parse_args()
    if args.live:
        verify_live()
    elif args.serial:
        verify_offline(args.serial)
    else:
        for s in (C.SERIAL_EYE_IN_HAND, C.SERIAL_EYE_TO_HAND):
            if load_result(s) is not None:
                verify_offline(s)
                print()


if __name__ == "__main__":
    main()
