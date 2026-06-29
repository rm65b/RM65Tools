# coding=utf-8

"""
手眼标定精度验证程序

读取 compute.py 保存的标定结果 (hand_eye_result.npz)，把相机坐标系下的一个点
转换到机械臂基坐标系，控制机械臂先到正上方、再垂直下放，使夹爪尖端到达该点附近，
并报告尖端实际位置与目标的偏差，以此验证手眼标定精度。

支持两种模式:
  eye_to_hand(眼在手外): p_base = R_cam2base @ p_cam + t_cam2base
  eye_in_hand(眼在手上): 读取当前末端位姿,
                         p_base = T_end2base @ (R_cam2end @ p_cam + t_cam2end)

接近姿态:
  默认沿用机械臂"当前姿态"（不强制夹爪垂直，运动为纯平移）；
  可用 --rpy rx ry rz(弧度) 指定接近姿态覆盖默认。
  夹爪尖端 = 法兰 + gripper * (姿态的 z 轴)，据此反推法兰目标。

用法:
  # 默认(用当前姿态接近):
  python validate.py --cam -0.027 -0.007 0.814 --gripper 0.20
  # 只算不动(预览):
  python validate.py --cam -0.027 -0.007 0.814 --gripper 0.20 --dry-run
  # 指定接近姿态(如夹爪朝下 rx=π):
  python validate.py --cam -0.027 -0.007 0.814 --gripper 0.20 --rpy 3.1416 0 0

可选: --clearance 0.005(尖端停在物体上方距离,米) --above 0.10(先到的正上方高度,米)
      --speed 20(运动速度%) --ip 192.168.1.18 --port 8080；不带 --cam 则交互式输入
"""

import os
import sys
import json
import time
import argparse
import socket

import numpy as np
from scipy.spatial.transform import Rotation as Rot

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
RESULT_FILE = os.path.join(BASE_DIR, "hand_eye_result.npz")


class ArmClient:
    """睿尔曼 JSON 协议(TCP)最小客户端：发指令、读状态、运动并轮询到位。"""

    def __init__(self, ip, port, timeout=30):
        self.c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.c.settimeout(timeout)
        self.c.connect((ip, port))

    def _parse(self, buf):
        objs, dec, i = [], json.JSONDecoder(), 0
        while i < len(buf):
            while i < len(buf) and buf[i].isspace():
                i += 1
            if i >= len(buf):
                break
            try:
                o, j = dec.raw_decode(buf[i:])
                objs.append(o)
                i += j
            except json.JSONDecodeError:
                break
        return objs, buf[i:]

    def readall(self, t=3.0):
        """读取 t 秒内收到的所有完整 JSON 对象。"""
        buf, got, dl = "", [], time.time() + t
        while time.time() < dl:
            try:
                self.c.settimeout(max(0.1, dl - time.time()))
                ch = self.c.recv(4096).decode()
            except socket.timeout:
                break
            if not ch:
                break
            buf += ch
            objs, buf = self._parse(buf)
            got += objs
        return got

    def send(self, obj):
        self.c.send(json.dumps(obj).encode())

    def cur(self):
        self.send({"command": "get_current_arm_state"})
        time.sleep(0.15)
        st = [o for o in self.readall(1.0) if o.get("state") == "current_arm_state"]
        return st[-1]["arm_state"] if st else None

    def cur_pose(self):
        a = self.cur()
        pr = a["pose"]
        return np.array(pr[:3]) / 1e6, np.array(pr[3:]) / 1e3  # pos(m), rpy(rad)

    def clear_err(self):
        self.send({"command": "clear_arm_err"})
        self.readall(1.0)

    def set_base(self):
        self.send({"command": "set_change_work_frame", "frame_name": "Base"})
        self.readall(1.0)

    def check_err(self):
        a = self.cur()
        if a["err"] != [0]:
            raise RuntimeError(f"机械臂报错未清除: {a['err']}")

    def movel_settled(self, pose_units, v=20, timeout=25.0):
        """
        下发 movel 并等待到位。
        第三代控制器 TCP 不返回 current_trajectory_state，故用「轮询位姿直到收敛」判断到位。
        """
        self.send({"command": "movel", "pose": pose_units, "v": v, "r": 0, "trajectory_connect": 0})
        got = self.readall(2.0)
        recv = [o.get("receive_state") for o in got if "receive_state" in o]
        if recv and recv[-1] is False:
            raise RuntimeError(f"movel 指令被拒(目标位姿可能超臂展/奇异): pose={pose_units}")
        prev, t0 = None, time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.5)
            p, _ = self.cur_pose()
            if prev is not None and np.linalg.norm(p - prev) < 1e-4:  # 0.1mm 内不再变化视为到位
                return p
            prev = p
        return p  # 超时返回最后位姿

    def close(self):
        try:
            self.c.close()
        except Exception:
            pass


def pose_to_T(pos, rpy):
    T = np.eye(4)
    T[:3, :3] = Rot.from_euler('xyz', rpy).as_matrix()
    T[:3, 3] = pos
    return T


def main():
    ap = argparse.ArgumentParser(description="手眼标定精度验证：移动夹爪到相机系下的点")
    ap.add_argument("--cam", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="相机坐标系下的目标点(米)")
    ap.add_argument("--gripper", type=float, default=0.20, help="夹爪长度(米), 默认0.20")
    ap.add_argument("--clearance", type=float, default=0.005,
                    help="尖端停在物体上方的距离(米), 默认0.005")
    ap.add_argument("--above", type=float, default=0.10, help="先到正上方的高度(米), 默认0.10")
    ap.add_argument("--speed", type=int, default=20, help="运动速度百分比, 默认20")
    ap.add_argument("--rpy", nargs=3, type=float, default=None, metavar=("RX", "RY", "RZ"),
                    help="接近姿态(弧度), 默认沿用当前姿态(不强制夹爪垂直)")
    ap.add_argument("--ip", default="192.168.1.18")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--dry-run", action="store_true", help="只计算并打印目标, 不运动")
    args = ap.parse_args()

    cam = args.cam
    if cam is None:
        s = input("相机坐标系点(x y z, 米, 空格分隔): ").split()
        cam = [float(v) for v in s]
    p_cam = np.array(cam, dtype=float)

    # 1) 读取标定结果
    if not os.path.exists(RESULT_FILE):
        print(f"找不到标定结果文件 {RESULT_FILE}，请先运行 compute.py。")
        sys.exit(1)
    d = np.load(RESULT_FILE)
    R_he, t_he, mode = d["R"], d["t"], str(d["mode"])
    print(f"标定结果模式: {mode}")

    # 2) 按需读取当前位姿：eye_in_hand 计算 p_base 必需；接近姿态默认沿用当前也必需
    arm = None
    cur_pos = cur_rpy = None
    need_pose = (mode == "eye_in_hand") or (args.rpy is None)
    if need_pose:
        arm = ArmClient(args.ip, args.port)
        arm.clear_err(); arm.set_base(); arm.check_err()
        cur_pos, cur_rpy = arm.cur_pose()

    # 3) 计算目标在基座系下的位置
    if mode == "eye_to_hand":
        p_base = R_he @ p_cam + t_he
    elif mode == "eye_in_hand":
        T_eb = pose_to_T(cur_pos, cur_rpy)
        p_base = T_eb[:3, :3] @ (R_he @ p_cam + t_he) + T_eb[:3, 3]   # 末端 -> 基座
    else:
        print(f"未知标定模式: {mode}")
        sys.exit(1)

    # 4) 接近姿态：--rpy 指定 > 当前姿态(不强制夹爪垂直)
    approach_rpy = list(args.rpy) if args.rpy is not None else list(cur_rpy)
    tool_z = Rot.from_euler('xyz', approach_rpy).as_matrix()[:, 2]

    # 尖端目标 = 物体上方 clearance(基座系 z); 法兰 = 尖端目标 - gripper*(姿态z轴)
    tip_target = p_base + np.array([0.0, 0.0, args.clearance])
    flange_target = tip_target - args.gripper * tool_z
    flange_above = flange_target + np.array([0.0, 0.0, args.above])

    def pu(p):
        return [int(round(p[0] * 1e6)), int(round(p[1] * 1e6)), int(round(p[2] * 1e6)),
                int(round(approach_rpy[0] * 1e3)), int(round(approach_rpy[1] * 1e3)),
                int(round(approach_rpy[2] * 1e3))]

    print(f"相机系点 p_cam  = {p_cam}")
    print(f"基座系点 p_base = [{p_base[0]:.4f}, {p_base[1]:.4f}, {p_base[2]:.4f}]")
    print(f"接近姿态 rpy(rad) = [{approach_rpy[0]:.4f}, {approach_rpy[1]:.4f}, {approach_rpy[2]:.4f}]"
          f"  {'(指定)' if args.rpy is not None else '(当前)'}")
    print(f"正上方法兰 = [{flange_above[0]:.4f}, {flange_above[1]:.4f}, {flange_above[2]:.4f}]")
    print(f"下放法兰   = [{flange_target[0]:.4f}, {flange_target[1]:.4f}, {flange_target[2]:.4f}]"
          f"  (尖端停物体上方 {args.clearance * 1000:.0f}mm)")

    if args.dry_run:
        print("\n--dry-run：仅计算，未运动。")
        if arm is not None:
            arm.close()
        return

    # 5) 运动控制：上方 -> 下放
    if arm is None:
        arm = ArmClient(args.ip, args.port)
        arm.clear_err(); arm.set_base(); arm.check_err()
    print(f"\n起始位姿 pos = {arm.cur_pose()[0].round(4).tolist()}")
    print("① movel 到正上方 ...")
    arm.movel_settled(pu(flange_above), args.speed)
    print("② movel 下放 ...")
    arm.movel_settled(pu(flange_target), args.speed)

    # 6) 报告精度
    fin_pos, fin_rpy = arm.cur_pose()
    fin_tool_z = Rot.from_euler('xyz', fin_rpy).as_matrix()[:, 2]
    tip = fin_pos + args.gripper * fin_tool_z
    err = tip - p_base
    print(f"\n夹爪尖端实际 = [{tip[0]:.4f}, {tip[1]:.4f}, {tip[2]:.4f}]")
    print(f"目标 p_base  = [{p_base[0]:.4f}, {p_base[1]:.4f}, {p_base[2]:.4f}]")
    print(f"偏差: dx={err[0] * 1000:.1f}mm dy={err[1] * 1000:.1f}mm dz={err[2] * 1000:.1f}mm"
          f" (dz 含预留 {args.clearance * 1000:.0f}mm)")
    print(f"XY 距离 = {np.hypot(err[0], err[1]) * 1000:.1f}mm")
    arm.close()


if __name__ == "__main__":
    main()
