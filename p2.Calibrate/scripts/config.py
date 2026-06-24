# -*- coding: utf-8 -*-
"""
全局配置：相机 SN、CharUco 板、RM65-B 本体参数（关节限位/奇异阈值）、
位姿约定、数据存档路径。

所有脚本 import 本文件即可拿到统一配置，集中改这里即可。
"""
from pathlib import Path
import numpy as np

# ----------------------------------------------------------------------------
# 相机
# ----------------------------------------------------------------------------
SERIAL_EYE_IN_HAND = "261722075459"   # 装在机械臂末端 → 眼在手上
SERIAL_EYE_TO_HAND = "261722076078"   # 固定俯视        → 眼在手外

CAM_RESOLUTION = (1280, 720)   # (width, height)，D435 彩色流
CAM_FPS = 30

# ----------------------------------------------------------------------------
# 机械臂 RM65-B
# ----------------------------------------------------------------------------
ARM_IP = "192.168.1.18"
ARM_PORT = 8080

# RM65-B 关节运动范围（°），取自官方本体参数页
JOINT_LIMITS_DEG = {
    1: (-178, 178),
    2: (-130, 130),
    3: (-135, 135),
    4: (-178, 178),
    5: (-128, 128),
    6: (-360, 360),
}

# 奇异点规避阈值（°）：肘部 q3=0、腕部 q5=0、边界 q3=q5=0；建议远离 0
SINGULARITY_MIN_ABS_DEG = 20.0   # |q3|、|q5| 应大于该值

# RM65-B 末端法兰 d6 = 144 mm（MDH 第 6 轴 di）。SDK 上报的 TCP 原点须落在该法兰面中心。
FLANGE_D6_MM = 144.0

# ----------------------------------------------------------------------------
# 位姿约定（关键，易错）
# ----------------------------------------------------------------------------
# RealMan movep/get_current_arm_state 的 pose = [x, y, z, rx, ry, rz]，
# (rx, ry, rz) 为 RPY 角。这里默认：
#   POSE_RPY_UNIT  = 'rad'   （Python SDK rm_movep 用弧度；若你的版本返回/接收度，改 'deg'）
#   POSE_RPY_ORDER = 'xyz'   （外旋 x→y→z，等价内旋 zyx；R = Rz(rz) @ Ry(ry) @ Rx(rx)）
# 若验证阶段出现系统性姿态偏差，先切换这两项再排查。
POSE_RPY_UNIT = "rad"
POSE_RPY_ORDER = "xyz"   # 外旋顺序

# ----------------------------------------------------------------------------
# CharUco 标定板（务必实测并改这两项：单位米）
# ----------------------------------------------------------------------------
BOARD_SQUARES_X = 7
BOARD_SQUARES_Y = 5
BOARD_SQUARE_LEN_M = 0.030   # 黑/白方格边长（米）
BOARD_MARKER_LEN_M = 0.022   # 内嵌 ArUco 标记边长（米）
BOARD_DICT_ID = None         # None → 取 ARUCO_DICT 预定义字典
ARUCO_DICT = "DICT_5X5_100"  # 预定义字典名（见 ARUCO_DICTS）

# 端到端对象点用方格内角点（CharUco 的棋盘交点），坐标已由板对象给出。
# 解算要求：单帧至少检测到这么多角点才采信
MIN_CHARUCO_CORNERS = 8

# ----------------------------------------------------------------------------
# 采集
# ----------------------------------------------------------------------------
SETTLE_TIME_S = 1.0          # 到位后停稳时间，再触发相机，避免位姿-图像错位
N_POSES_DEFAULT = 18         # 每路建议采集位姿数
AX_XB_MIN_PAIRS = 8          # 解算 AX=XB 至少需要的位姿对数

# ----------------------------------------------------------------------------
# 路径
# ----------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent.parent   # p2.Calibrate/
CAM_DIR = ROOT / "cameras"

def cam_root(serial: str) -> Path:
    return CAM_DIR / serial

def samples_dir(serial: str) -> Path:
    return cam_root(serial) / "samples"

def intrinsics_path(serial: str) -> Path:
    return cam_root(serial) / "intrinsics.json"

def handeye_path(serial: str) -> Path:
    """眼在手上 → handeye.json(存 T_flange_cam)；眼在手外 → eye_to_hand.json(存 T_base_cam)"""
    return cam_root(serial) / "handeye.json"

# 预定义字典映射（OpenCV 4.13 新 API）
ARUCO_DICTS = {
    "DICT_4X4_50": 0, "DICT_4X4_100": 1, "DICT_4X4_250": 2, "DICT_4X4_1000": 3,
    "DICT_5X5_50": 4, "DICT_5X5_100": 5, "DICT_5X5_250": 6, "DICT_5X5_1000": 7,
    "DICT_6X6_50": 8, "DICT_6X6_100": 9, "DICT_6X6_250": 10, "DICT_6X6_1000": 11,
    "DICT_7X7_50": 12, "DICT_7X7_100": 13, "DICT_7X7_250": 14, "DICT_7X7_1000": 15,
}

# 标定阶段名称（写进存档）
CALIB_METHODS = ["TSAI", "PARK", "DANIILIDIS", "ANDREFF"]
METHOD_CV2 = {
    "TSAI": "CALIB_HAND_EYE_TSAI",
    "PARK": "CALIB_HAND_EYE_PARK",
    "DANIILIDIS": "CALIB_HAND_EYE_DANIILIDIS",
    "ANDREFF": "CALIB_HAND_EYE_ANDREFF",
}
