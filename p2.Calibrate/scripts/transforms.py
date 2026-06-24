# -*- coding: utf-8 -*-
"""
齐次变换与旋转工具（不依赖 scipy）。

约定：所有 4x4 齐次矩阵 T_X_Y 表示“把 Y 系下的点变到 X 系”：P_X = T_X_Y @ P_Y。
位姿 pose = [x, y, z, rx, ry, rz]，旋转部分为 RPY，约定见 config。
"""
import numpy as np


# ----------------------------- 基本旋转矩阵 ---------------------------------
def _Rx(a):  # NOQA
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)

def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)

def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def rpy_to_rotmat(rpy, order="xyz"):
    """
    外旋（extrinsic）顺序。order 为小写轴序，按从左到右依次绕固定轴旋转。
    默认 'xyz'：R = Rz(rz) @ Ry(ry) @ Rx(rx)，rpy=(rx,ry,rz)。
    """
    rx, ry, rz = np.asarray(rpy, dtype=np.float64).reshape(-1)
    # 外旋：先绕 x → 再绕 y → 再绕 z，矩阵右乘序列反序
    funcs = {"x": (_Rx, rx), "y": (_Ry, ry), "z": (_Rz, rz)}
    R = np.eye(3)
    for axis in reversed(order):
        f, ang = funcs[axis]
        R = R @ f(ang)
    return R


def rotmat_to_rpy(R, order="xyz"):
    """外旋 xyz 的逆解：返回 (rx, ry, rz)。"""
    R = np.asarray(R, dtype=np.float64)
    if order != "xyz":
        raise NotImplementedError("仅实现外旋 xyz 顺序的逆解")
    # R = Rz(rz) @ Ry(ry) @ Rx(rx) 的解析式
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0.0
    return np.array([rx, ry, rz], dtype=np.float64)


# ----------------------------- pose <-> 4x4 ---------------------------------
def pose_to_mat4(pose, unit="rad", order="xyz"):
    """
    pose=[x,y,z,rx,ry,rz] → 4x4 齐次矩阵（T_base_flange 或 T_cam_board 均可）。
    unit: 'rad' 或 'deg'。
    """
    pose = np.asarray(pose, dtype=np.float64).reshape(-1)
    if pose.size < 6:
        raise ValueError(f"pose 需 6 维，得到 {pose.size}")
    t = pose[:3]
    rpy = pose[3:6]
    if unit == "deg":
        rpy = np.deg2rad(rpy)
    elif unit != "rad":
        raise ValueError(f"unit 需 'rad'/'deg'，得到 {unit}")
    T = np.eye(4)
    T[:3, :3] = rpy_to_rotmat(rpy, order)
    T[:3, 3] = t
    return T


def mat4_to_pose(T, unit="rad", order="xyz"):
    """4x4 → [x,y,z,rx,ry,rz]。"""
    T = np.asarray(T, dtype=np.float64)
    rpy = rotmat_to_rpy(T[:3, :3], order)
    if unit == "deg":
        rpy = np.rad2deg(rpy)
    return np.concatenate([T[:3, 3], rpy])


# ----------------------------- 矩阵运算 -------------------------------------
def invert(T):
    """齐次矩阵求逆（比 np.linalg.inv 高效、稳定）。"""
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    inv = np.eye(4)
    inv[:3, :3] = R.T
    inv[:3, 3] = -R.T @ t
    return inv


def compose(*mats):
    """左乘链：compose(A, B, C) == A @ B @ C，语义为“先 C 再 B 再 A”。"""
    out = np.eye(4)
    for m in mats:
        out = out @ np.asarray(m, dtype=np.float64)
    return out


# ----------------------------- 残差度量 -------------------------------------
def logmap_rot(R):
    """SO(3) 对数映射 → 旋转向量(3,)，其模长即旋转角(弧度)。"""
    R = np.asarray(R, dtype=np.float64)
    cos_angle = np.clip((np.trace(R) - 1) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    if angle < 1e-8:
        return np.zeros(3)
    if np.abs(angle - np.pi) < 1e-6:
        # 接近 π，用对称矩阵恢复
        # R = I*sin(angle) ... 退化情形，用矩阵平方根法
        # 这里取最大的反对称分量近似
        tmp = R + np.eye(3)
        idx = np.argmax(np.diagonal(tmp))
        v = tmp[:, idx]
        v = v / np.linalg.norm(v)
        return v * angle
    skew = (R - R.T) / (2.0 * np.sin(angle))
    vec = np.array([skew[2, 1], skew[0, 2], skew[1, 0]]) * angle
    return vec


def chain_consistency(T_list):
    """
    一组 4x4 变换的跨样本离散度 → (rot_rms_deg, trans_rms_mm)。
    手眼标定中：若 X 正确，则物理常量链（板在基座/板在法兰）应跨样本不变，
    其离散度即为标定残差。
    """
    rots = np.array([logmap_rot(T[:3, :3]) for T in T_list])
    trans = np.array([T[:3, 3] for T in T_list])
    rot_rms = np.sqrt(np.mean(np.sum((rots - rots.mean(0)) ** 2, axis=1)))
    trans_rms = np.sqrt(np.mean(np.sum((trans - trans.mean(0)) ** 2, axis=1)))
    return float(np.rad2deg(rot_rms)), float(trans_rms * 1000.0)


def is_orthonormal(R, tol=1e-4):
    R = np.asarray(R, dtype=np.float64)
    return np.allclose(R @ R.T, np.eye(3), atol=tol) and abs(np.linalg.det(R) - 1) < tol
