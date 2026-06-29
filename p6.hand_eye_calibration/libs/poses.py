# coding=utf-8

"""
统一的机械臂位姿 -> 齐次变换矩阵 转换模块（合并原 save_poses.py / save_poses2.py）。

- 眼在手上 (eye_in_hand)：直接用 机械臂末端相对基座 的齐次矩阵（invert=False）。
- 眼在手外 (eye_to_hand)：用其逆，即 基座相对末端 的齐次矩阵（invert=True）。
"""

import csv

import numpy as np


def euler_angles_to_rotation_matrix(rx, ry, rz):
    """欧拉角(rx,ry,rz) -> 旋转矩阵，按 Z-Y-X 顺序。"""
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])

    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])

    R = Rz @ Ry @ Rx  # 先绕 x 轴，再绕 y 轴，最后绕 z 轴
    return R


def pose_to_homogeneous_matrix(pose):
    """位姿 (x,y,z,rx,ry,rz) -> 4x4 齐次变换矩阵。"""
    x, y, z, rx, ry, rz = pose
    R = euler_angles_to_rotation_matrix(rx, ry, rz)
    t = np.array([x, y, z]).reshape(3, 1)

    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t[:, 0]

    return H


def inverse_transformation_matrix(T):
    """齐次变换矩阵求逆（旋转用转置，平移相应变换）。"""
    R = T[:3, :3]
    t = T[:3, 3]

    R_inv = R.T
    t_inv = -np.dot(R_inv, t)

    T_inv = np.identity(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv

    return T_inv


def save_matrices_to_csv(matrices, file_name):
    """把多个 4x4 矩阵横向拼接后保存为 CSV（每个矩阵占固定的 4 列）。"""
    rows, cols = matrices[0].shape
    num_matrices = len(matrices)
    combined_matrix = np.zeros((rows, cols * num_matrices))

    for i, matrix in enumerate(matrices):
        combined_matrix[:, i * cols: (i + 1) * cols] = matrix

    with open(file_name, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        for row in combined_matrix:
            csv_writer.writerow(row)


def poses_to_csv(filepath, out_file_name, invert=False):
    """
    读取 poses.txt（每行 x,y,z,rx,ry,rz）-> 齐次变换矩阵 -> 保存为 CSV。

    参数:
        filepath: poses.txt 路径
        out_file_name: 输出 CSV 文件名
        invert: False=眼在手上(末端相对基座)；True=眼在手外(对其求逆，基座相对末端)
    """
    with open(filepath, "r", encoding="utf-8") as f:
        lines = f.readlines()

    vals = [float(i) for line in lines for i in line.split(',')]

    matrices = []
    for i in range(0, len(vals), 6):
        H = pose_to_homogeneous_matrix(vals[i:i + 6])
        if invert:
            H = inverse_transformation_matrix(H)
        matrices.append(H)

    save_matrices_to_csv(matrices, out_file_name)
