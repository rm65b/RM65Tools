# -*- coding: utf-8 -*-
"""
01 · 采集 (eye_in_hand / eye_to_hand)。

实时预览（默认）：打开图像窗口，连续显示画面 + CharUco 角点叠层 + 状态提示
（绿“OK 角点数/板距”或红“ADJUST 角点不足”），操作者摆好位姿、看到角点全绿后按
【空格/回车】采集，【q/ESC】结束保存。这样可在采集前确认标定板是否在视野内。
无图形界面(--headless)时回退为输入提示式。

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

    board_hint = ("标定板固定在桌面；摆动末端相机，使板始终在视野内" if args.mode == "eye_in_hand"
                  else "标定板夹在末端夹爪(随臂动)；俯视相机 B 固定，板在视野内")
    print(f"\n布板：{board_hint}")
    print("位姿要求：绕 ≥2 个不平行轴旋转，|q3|、|q5|>20°，板距 0.3–0.8m。")

    def overlay(frame, corners, ids, n_done, n_tgt, dist_mm, warn):
        vis = draw_detected(frame, corners, ids) if corners is not None else frame.copy()
        n = len(corners) if corners is not None else 0
        ok = n >= C.MIN_CHARUCO_CORNERS
        msg = (f"OK  {n} pts  (dist~{dist_mm:.0f}mm)" if ok
               else f"ADJUST  {n}/{C.MIN_CHARUCO_CORNERS} pts")
        col = (0, 200, 0) if ok else (0, 0, 255)
        cv2.rectangle(vis, (0, 0), (vis.shape[1], 60), (0, 0, 0), -1)
        cv2.putText(vis, msg, (16, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.1, col, 3)
        cv2.putText(vis, f"captured {n_done}/{n_tgt}", (vis.shape[1] - 230, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(vis, "[Space/Enter]capture   [s]skip   [q/ESC]quit",
                    (16, vis.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)
        if warn:
            cv2.putText(vis, "WARN: " + warn, (16, 88),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2)
        return vis

    def do_capture(color_cap, corners_cap, ids_cap, ts_cap):
        nonlocal idx
        joints, pose = arm.get_state()
        ok_s, msg_s = check_singularity(joints)
        ok_l, msg_l = check_joint_limits(joints)
        if not ok_s:
            print("  [!] 奇异风险:", msg_s)
        if not ok_l:
            print("  [!] 关节超限:", msg_l)
        T_cam_board = estimate_pose(board, corners_cap, ids_cap, intr)
        meta = {
            "idx": idx, "serial": serial, "mode": args.mode,
            "pose": [float(v) for v in pose],
            "joints_deg": [float(v) for v in joints],
            "pose_unit": C.POSE_RPY_UNIT, "pose_order": C.POSE_RPY_ORDER,
            "n_corners": int(len(corners_cap)), "timestamp": float(ts_cap),
        }
        img_path, meta_path = save_sample(out_dir, idx, color_cap, meta)
        cv2.imwrite(str(out_dir / f"{idx:04d}_vis.png"),
                    draw_detected(color_cap, corners_cap.reshape(-1, 1, 2), ids_cap))
        samples.append({
            "idx": idx, "image": str(img_path.relative_to(C.ROOT)),
            "meta": str(meta_path.relative_to(C.ROOT)),
            "pose": meta["pose"], "joints_deg": meta["joints_deg"],
            "n_corners": meta["n_corners"],
        })
        print(f"  [v] #{idx}: 角点={len(corners_cap)} 板距≈{np.linalg.norm(T_cam_board[:3,3])*1000:.0f}mm")
        idx += 1

    if args.headless:
        # 无图形界面回退：输入提示式（看不到画面）
        tried = 0
        while len(samples) < args.n:
            print(f"\n--- 第 {len(samples)+1}/{args.n} 个有效样本 (已尝试 {tried}) ---")
            cmd = input("回车=采集  s=跳过  q=结束: ").strip().lower()
            if cmd == "q":
                break
            if cmd == "s":
                tried += 1
                continue
            time.sleep(C.SETTLE_TIME_S)
            color, _d, ts = cam.grab()
            corners, ids = detect(detector, color)
            if corners is None:
                print(f"  [x] 角点不足(<{C.MIN_CHARUCO_CORNERS})，丢弃重试。")
                tried += 1
                continue
            do_capture(color, corners, ids, ts)
    else:
        # 实时预览：连续显示画面+检测叠层，按键采集
        print("\n[实时预览] 图像窗口已打开。摆好姿态、角点全绿后按【空格/回车】采集；q/ESC 结束保存。")
        win = "collect-live  (Space=采集  s=跳过  q/ESC=结束)"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        while len(samples) < args.n:
            color, _d, ts = cam.grab(warmup_frames=0)
            corners, ids = detect(detector, color)
            n = len(corners) if corners is not None else 0
            ok = corners is not None and n >= C.MIN_CHARUCO_CORNERS
            if corners is not None:
                dist_mm = np.linalg.norm(estimate_pose(board, corners, ids, intr)[:3, 3]) * 1000
            else:
                dist_mm = 0.0
            warn = ""
            try:   # 实时奇异/限位预警（轻量读取，失败不中断）
                joints, _ = arm.get_state()
                _s, ms = check_singularity(joints)
                _l, ml = check_joint_limits(joints)
                warn = "  ".join(ms + ml)
            except Exception:
                pass
            cv2.imshow(win, overlay(color, corners, ids, len(samples), args.n, dist_mm, warn))
            key = cv2.waitKey(25) & 0xFF
            if key in (ord("q"), 27):            # q / ESC
                break
            if key in (ord(" "), 13, 10):        # 空格 / 回车
                if not ok:
                    print("  [x] 角点不足，未保存；调整后重按空格。")
                    continue
                time.sleep(0.3)                  # 等手松开/停稳
                color2, _d2, ts2 = cam.grab(warmup_frames=1)
                c2, i2 = detect(detector, color2)
                if c2 is None:
                    print("  [x] 采集瞬间检测失败，未保存。")
                    continue
                do_capture(color2, c2, i2, ts2)

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
