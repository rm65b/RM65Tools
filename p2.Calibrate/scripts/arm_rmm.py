# -*- coding: utf-8 -*-
"""
机械臂控制层（对接 RealMan 官方 Python SDK `Robotic_Arm` v1.1.5）。

API 事实（经源码 + gitee RM_API2 README 核对）：
  from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
  robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)        # 构造传线程模式，型号连接时自识别
  handle = robot.rm_create_robot_arm(ip, 8080)                  # handle.id==-1 表示失败
  code, state_dict = robot.rm_get_current_arm_state()           # state_dict['pose']=[x,y,z,rx,ry,rz](欧拉rad)
                                                                # state_dict['joint']  关节角(度)
  robot.rm_movej(joint[6], v, r, connect, block)                # 关节运动
  robot.rm_movel(pose[6],  v, r, connect, block)                # 笛卡尔直线运动(无 rm_movep)
  robot.rm_set_gripper_pick_on(speed, force, block, timeout)    # CTAG2F90D 夹取
  robot.rm_set_gripper_release(speed, block, timeout)           # 松开

位姿约定：欧拉角 rx,ry,rz 为绕 X/Y/Z 轴角度、单位 rad，
旋转顺序 R = Rz(rz)·Ry(ry)·Rx(rx)（外旋 xyz），与 config.POSE_RPY_* 默认一致。

- RealManArm : 对接上述 SDK。
- ManualArm  : 无 SDK 回退，操作者用示教器摆位后粘贴法兰位姿。
"""
import sys
import numpy as np

import config as C


# ----------------------------- 安全校验（RM65-B） ----------------------------
def check_joint_limits(joints_deg):
    """返回 (ok, msgs)。关节超限告警。"""
    msgs = []
    j = np.asarray(joints_deg, dtype=np.float64).reshape(-1)
    for i in range(min(6, j.size)):
        lo, hi = C.JOINT_LIMITS_DEG[i + 1]
        if j[i] < lo or j[i] > hi:
            msgs.append(f"J{i+1}={j[i]:.1f}° 超限 [{lo},{hi}]°")
    return (len(msgs) == 0, msgs)


def check_singularity(joints_deg):
    """
    RM65-B 四类奇异：肘部 q3≈0、腕部 q5≈0、边界 q3=q5≈0、肩部奇异。
    返回 (ok, msgs)。|q3|、|q5| 应 > SINGULARITY_MIN_ABS_DEG。
    """
    msgs = []
    j = np.asarray(joints_deg, dtype=np.float64).reshape(-1)
    if j.size >= 5:
        q3, q5 = j[2], j[4]
        if abs(q3) < C.SINGULARITY_MIN_ABS_DEG:
            msgs.append(f"q3={q3:.1f}° 接近肘部/边界奇异（应 |q3|>{C.SINGULARITY_MIN_ABS_DEG}°）")
        if abs(q5) < C.SINGULARITY_MIN_ABS_DEG:
            msgs.append(f"q5={q5:.1f}° 接近腕部/边界奇异（应 |q5|>{C.SINGULARITY_MIN_ABS_DEG}°）")
    return (len(msgs) == 0, msgs)


# ----------------------------- RealMan SDK 适配器 ----------------------------
class RealManArm:
    """对接 Robotic_Arm.rm_robot_interface.RoboticArm。"""

    def __init__(self, ip=None, port=None, thread_mode=None):
        self.ip = ip or C.ARM_IP
        self.port = port or C.ARM_PORT
        self.thread_mode = thread_mode
        self.arm = None
        self.handle = None

    def connect(self):
        from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e   # pip install Robotic_Arm
        mode = self.thread_mode or rm_thread_mode_e.RM_TRIPLE_MODE_E
        self.arm = RoboticArm(mode)
        self.handle = self.arm.rm_create_robot_arm(self.ip, self.port)
        # 失败判定：handle.id == -1（不同包装下可能在 .id 或 .contents.id）
        arm_id = None
        for attr in ("id",):
            try:
                arm_id = getattr(self.handle, attr, None)
            except Exception:
                arm_id = None
        if arm_id is None:
            try:
                arm_id = self.handle.contents.id
            except Exception:
                arm_id = None
        connected = arm_id is not None and arm_id != -1
        # 兜底：用一次状态读取确认链路
        if not connected:
            try:
                code, _ = self.arm.rm_get_current_arm_state()
                connected = (code == 0)
            except Exception:
                connected = False
        if not connected:
            raise RuntimeError(f"机械臂连接失败 ip={self.ip}:{self.port}（handle.id={arm_id}）")
        print(f"[arm] 已连接 RM65 @ {self.ip}:{self.port} (id={arm_id})")
        return self

    # ---- 原始状态 ----
    def raw_state(self):
        """返回 (code, state_dict)。dict['pose']=[x,y,z,rx,ry,rz](rad), dict['joint']=关节角(度)。"""
        return self.arm.rm_get_current_arm_state()

    def get_state(self):
        """
        返回 (joints_deg[6], pose_xyzrpy[6])。
        pose 欧拉角为 rad，关节角为度（用于奇异/限位检查）。
        """
        code, d = self.raw_state()
        if code != 0:
            raise RuntimeError(f"rm_get_current_arm_state 失败 code={code}（0=成功）")
        pose = np.asarray(d["pose"], dtype=np.float64).reshape(6)         # x,y,z,rx,ry,rz(rad)
        joints = np.asarray(d.get("joint", []), dtype=np.float64).reshape(-1)
        if joints.size < 6:
            joints = np.zeros(6)
        return joints, pose

    # ---- 运动 ----
    def movej(self, joints_deg, v=20, block=True):
        """关节运动。joints_deg 单位度，v 关节速度。"""
        code = self.arm.rm_movej([float(x) for x in joints_deg], int(v), 0, 0, 1 if block else 0)
        if code != 0:
            raise RuntimeError(f"rm_movej 失败 code={code}")
        return code

    def movel(self, pose, v=20, block=True):
        """笛卡尔直线运动（SDK 无 rm_movep，用 rm_movel）。pose 欧拉角 rad。"""
        code = self.arm.rm_movel([float(x) for x in pose], int(v), 0, 0, 1 if block else 0)
        if code != 0:
            raise RuntimeError(f"rm_movel 失败 code={code}")
        return code

    # ---- 夹爪 CTAG2F90D（两指电动夹爪） ----
    def gripper_open(self, speed=200, timeout=5):
        """张开（运动到开口最大）。"""
        return self.arm.rm_set_gripper_release(int(speed), True, int(timeout))

    def gripper_close(self, speed=200, force=50, timeout=5):
        """力控夹合（用于在末端固定标定板）。"""
        return self.arm.rm_set_gripper_pick_on(int(speed), int(force), True, int(timeout))

    def close(self):
        try:
            self.arm.rm_delete_robot_arm()
        except Exception:
            pass


# ----------------------------- 手动回退（无 SDK） ----------------------------
class ManualArm:
    """
    无 RealMan SDK 时的回退：操作者用示教器/RM App 摆位，按提示粘贴
    当前法兰位姿 [x,y,z,rx,ry,rz]（欧拉 rad），
    可选再加 6 个关节角(度) 用于奇异检查。
    """

    def __init__(self, infile=None):
        self.infile = infile

    def connect(self):
        print("[arm] 手动模式：用示教器/RM App 摆位，按提示粘贴法兰位姿。")
        return self

    def get_state(self):
        prompt = (f"粘贴当前法兰位姿 [x,y,z,rx,ry,rz]（欧拉单位={C.POSE_RPY_UNIT}），"
                  "可选再加 6 关节角(度):\n> ")
        if self.infile is not None:
            line = self.infile.readline()
            if not line:
                raise EOFError("位姿文件已读完")
        else:
            line = input(prompt)
        vals = [float(v) for v in line.replace(",", " ").split()]
        if len(vals) < 6:
            raise ValueError("至少需要 6 个数 [x,y,z,rx,ry,rz]")
        pose = np.array(vals[:6], dtype=np.float64)
        joints = np.array(vals[6:12], dtype=np.float64) if len(vals) >= 12 else np.zeros(6)
        return joints, pose

    def close(self):
        pass


# ----------------------------- 工厂 -----------------------------------------
def make_arm(method="sdk", ip=None, port=None, infile=None):
    """
    method:
      'sdk'    → RealManArm（要求已 pip install Robotic_Arm）
      'manual' → ManualArm
    """
    if method == "manual":
        return ManualArm(infile=infile).connect()
    try:
        return RealManArm(ip=ip, port=port).connect()
    except ImportError:
        print("[arm] 未检测到 Robotic_Arm，自动回退到手动模式。\n"
              "      安装 SDK:  pip install Robotic_Arm", file=sys.stderr)
        return ManualArm(infile=infile).connect()
