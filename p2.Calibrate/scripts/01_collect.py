# -*- coding: utf-8 -*-
"""
01 · 采集 (eye_in_hand / eye_to_hand)。

半自动流程：操作者用示教器把机械臂摆到不同位姿（绕 ≥2 个不平行轴旋转、
规避 RM65-B 奇异点），到位停稳后按回车，脚本自动：
  1. 读取当前法兰位姿 [x,y,z,rx,ry,rz] → T_base_flange
  2. 触发一帧 RGB（已对齐深度）
  3. CharUco 检测；成功则保存 图像 + 位姿 + 检测信息

用法:
  python scripts/01_collect.py --mode eye_in_hand            # 相机 A(5459) 眼在手上
  python scripts/01_collect.py --mode eye_to_hand            # 相机 B(6078) 眼在手外
  python scripts/01_collect.py --mode eye_in_hand --arm manual   # 无 SDK，手动粘贴位姿
  python scripts/01_collect.py --mode eye_in_hand --inspect      # 仅打印位姿/内参自检
"""
import os
import sys
import json
import time
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np

import config as C
from realsense_cam import RealSenseCam, save_intrinsics
from charuco import make_board, detect, draw_detected, estimate_pose
from arm_rmm import make_arm, check_singularity, check_joint_limits
from transforms import pose_to_mat4


def serial_for(mode):
    return C.SERIAL_EYE_IN_HAND if mode == "eye_in_hand" else C.SERIAL_EYE_TO_HAND


def save_sample(out_dir, idx, color, meta):
    img_path = out_dir / f"{idx:04d}_color.png"
    cv2.imwrite(str(img_path), color)
    meta_path = out_dir / f"{idx:04d}_meta.json"
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    return img_path, meta_path


def run(args):
    serial = args.serial or serial_for(args.mode)
    board, detector = make_board()
    print(f"[collect] 模式={args.mode}  相机SN={serial}  目标位姿数={args.n}")

    # ---- 相机 ----
    cam = RealSenseCam(serial).start()
    intr = cam.intrinsics()
    if C.intrinsics_path(serial).exists():
        print(f"[cam] 内参已存在: {C.intrinsics_path(serial)}")
    else:
        p = save_intrinsics(serial, intr, source="factory")
        print(f"[cam] 已保存出厂内参: {p}")

    # ---- 机械臂 ----
    arm = make_arm(args.arm, ip=args.ip, port=args.port)

    if args.inspect:
        print("\n=== 自检 ===")
        try:
            joints, pose = arm.get_state()
            print(f"关节角(度) = {np.round(joints,2).tolist()}")
            print(f"法兰位姿 [{C.POSE_RPY_UNIT}] = {np.round(pose,4).tolist()}")
            ok_s, msg_s = check_singularity(joints)
            ok_l, msg_l = check_joint_limits(joints)
            print(f"奇异校验: {'OK' if ok_s else msg_s}")
            print(f"限位校验: {'OK' if ok_l else msg_l}")
        except Exception as e:
            print(f"[!] 位姿读取失败，请按报错调整 arm_rmm._parse_state: {e}")
        print(f"内参 fx={intr['fx']:.1f} fy={intr['fy']:.1f} ppx={intr['ppx']:.1f} ppy={intr['ppy']:.1f}")
        cam.stop(); arm.close()
        return

    out_dir = C.samples_dir(serial)
    out_dir.mkdir(parents=True, exist_ok=True)
    samples = []
    idx = 1
    tried = 0
    while len(samples) < args.n:
        print(f"\n--- 第 {len(samples)+1}/{args.n} 个有效样本 (已尝试 {tried}) ---")
        print("布板提示:")
        if args.mode == "eye_in_hand":
            print("  标定板固定在桌面不动；摆动末端相机，使板始终在视野内，")
        else:
            print("  标定板已固定在末端夹爪(随臂动)；俯视相机 B 不动，板在视野内，")
        print("  位姿需绕 ≥2 个不平行轴有旋转，|q3|、|q5|>20°。摆好后回车。")
        cmd = input("回车=采集  s=跳过本姿态  q=结束保存: ").strip().lower()
        if cmd == "q":
            break
        if cmd == "s":
            tried += 1
            continue

        # 等停稳后读位姿
        time.sleep(C.SETTLE_TIME_S)
        joints, pose = arm.get_state()

        # 安全校验（仅告警，不阻断）
        ok_s, msg_s = check_singularity(joints)
        ok_l, msg_l = check_joint_limits(joints)
        if not ok_s:
            print("  [!] 奇异风险:", msg_s)
        if not ok_l:
            print("  [!] 关节超限:", msg_l)

        # 采图 + 检测
        color, depth, ts = cam.grab()
        corners, ids = detect(detector, color)
        if corners is None:
            print(f"  [x] 未检测到足够 CharUco 角点(<{C.MIN_CHARUCO_CORNERS})，丢弃，换姿态重试。")
            tried += 1
            continue

        # 计算板位姿（即时反馈，解算阶段会重算）
        T_cam_board = estimate_pose(board, corners, ids, intr)
        meta = {
            "idx": idx,
            "serial": serial,
            "mode": args.mode,
            "pose": [float(v) for v in pose],
            "joints_deg": [float(v) for v in joints],
            "pose_unit": C.POSE_RPY_UNIT,
            "pose_order": C.POSE_RPY_ORDER,
            "n_corners": int(len(corners)),
            "timestamp": float(ts),
        }
        img_path, meta_path = save_sample(out_dir, idx, color, meta)
        samples.append({
            "idx": idx, "image": str(img_path.relative_to(C.ROOT)),
            "meta": str(meta_path.relative_to(C.ROOT)),
            "pose": meta["pose"], "joints_deg": meta["joints_deg"],
            "n_corners": meta["n_corners"],
        })
        # 可视化留档 + 实时预览
        vis = draw_detected(color, corners.reshape(-1, 1, 2), ids)
        cv2.imwrite(str(out_dir / f"{idx:04d}_vis.png"), vis)
        if not args.headless:
            cv2.imshow("collect", vis)
            cv2.waitKey(1)
        print(f"  [v] 已保存 #{idx}：角点={len(corners)}，板距≈{np.linalg.norm(T_cam_board[:3,3])*1000:.0f} mm")
        idx += 1

    # 汇总
    manifest = {
        "serial": serial, "mode": args.mode,
        "pose_unit": C.POSE_RPY_UNIT, "pose_order": C.POSE_RPY_ORDER,
        "board": {"squares": [C.BOARD_SQUARES_X, C.BOARD_SQUARES_Y],
                  "square_len_m": C.BOARD_SQUARE_LEN_M,
                  "marker_len_m": C.BOARD_MARKER_LEN_M,
                  "dict": C.ARUCO_DICT},
        "n": len(samples), "samples": samples,
    }
    mf = out_dir / "samples.json"
    mf.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n[done] 采集 {len(samples)} 个样本，清单: {mf}")
    if len(samples) < C.AX_XB_MIN_PAIRS:
        print(f"[!] 少于 {C.AX_XB_MIN_PAIRS} 个，解算可能不稳定，建议补采。")

    if not args.headless:
        cv2.destroyAllWindows()
    cam.stop(); arm.close()


def main():
    ap = argparse.ArgumentParser(description="CharUco 手眼/眼在手外数据采集")
    ap.add_argument("--mode", required=True, choices=["eye_in_hand", "eye_to_hand"])
    ap.add_argument("--serial", default=None, help="相机 SN；缺省按 mode 自动选")
    ap.add_argument("--n", type=int, default=C.N_POSES_DEFAULT, help="目标样本数")
    ap.add_argument("--arm", default="sdk", choices=["sdk", "manual"], help="机械臂驱动方式")
    ap.add_argument("--ip", default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--inspect", action="store_true", help="只做位姿/内参自检后退出")
    ap.add_argument("--headless", action="store_true", help="无图形界面，不弹窗")
    args = ap.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print("\n[中断]")
        sys.exit(130)


if __name__ == "__main__":
    main()
