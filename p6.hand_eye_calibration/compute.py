# coding=utf-8

"""
手眼标定计算（统一版，合并原 compute_in_hand.py / compute_to_hand.py）。

依据标定模式选择：
  - eye_in_hand 眼在手上：计算 相机坐标系 -> 机械臂末端坐标系 (R_cam2end, t_cam2end)，位姿不求逆
  - eye_to_hand 眼在手外：计算 相机坐标系 -> 机械臂基坐标系 (R_cam2base, t_cam2base)，位姿求逆
两种模式都归结为 AX=XB。

模式与数据夹的确定（优先级从高到低）：
  --mode / --data 命令行参数  >  数据夹内 mode.txt  >  config.yaml 的 calib_mode / 最新数据夹

用法:
  python compute.py                                  # 自动：最新数据夹 + mode.txt/config 模式
  python compute.py --data data20260629              # 指定数据夹，模式按 mode.txt/config
  python compute.py --data data20260629 --mode eye_in_hand   # 指定数据夹和模式
"""

import os
import logging
import argparse

import yaml
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from libs.auxiliary import find_latest_data_folder
from libs.log_setting import CommonLog
from libs.poses import poses_to_csv

np.set_printoptions(precision=8, suppress=True)

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)

DATA_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "eye_hand_data")

# 标定板参数
with open("config.yaml", "r", encoding="utf-8") as _file:
    _cfg = yaml.safe_load(_file)
XX = _cfg.get("checkerboard_args").get("XX")  # 标定板长度方向内角点数
YY = _cfg.get("checkerboard_args").get("YY")  # 标定板宽度方向内角点数
L = _cfg.get("checkerboard_args").get("L")    # 单格边长（米）


def resolve_images_path(data_arg=None):
    """确定数据夹：--data 指定 > 最新的 dataYYYYMMDD[NN] 夹。"""
    if data_arg:
        p = os.path.join("eye_hand_data", data_arg)
        if not os.path.isdir(p):
            raise FileNotFoundError(f"数据夹不存在: {p}")
        return p
    latest = find_latest_data_folder(DATA_ROOT)
    if not latest:
        raise FileNotFoundError(f"在 {DATA_ROOT} 下未找到任何数据夹")
    return os.path.join("eye_hand_data", latest)


def resolve_mode(images_path, mode_arg=None):
    """确定标定模式：--mode > 数据夹 mode.txt > config.yaml calib_mode。"""
    if mode_arg:
        return mode_arg
    mode_file = os.path.join(images_path, "mode.txt")
    if os.path.exists(mode_file):
        with open(mode_file, "r", encoding="utf-8") as f:
            return f.read().strip() or "eye_in_hand"
    return _cfg.get("calib_mode", "eye_in_hand")


def calibrate(images_path, invert):
    """
    用 images_path 下的标定板图片(.jpg)和机械臂位姿(poses.txt)做手眼标定。
    invert=False(眼在手上)位姿不求逆；invert=True(眼在手外)位姿求逆。
    返回 (R, t)。
    """
    path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(images_path, "poses.txt")  # 与图片顺序一一对应

    # 亚像素角点查找停止准则：最大循环 30 次，误差容限 0.001
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    # 标定板角点的世界坐标（建在标定板上，Z=0）
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)
    objp = L * objp

    obj_points = []     # 存储 3D 点
    img_points = []     # 存储 2D 点

    images_num = [f for f in os.listdir(images_path) if f.endswith('.jpg')]

    size = None
    for i in range(1, len(images_num) + 1):   # 图片从 1.jpg 到 x.jpg

        image_file = os.path.join(images_path, f"{i}.jpg")

        if os.path.exists(image_file):

            logger_.info(f'读 {image_file}')

            img = cv2.imread(image_file)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            size = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)

            if ret:
                obj_points.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 亚像素角点
                if [corners2]:
                    img_points.append(corners2)
                else:
                    img_points.append(corners)

    N = len(img_points)
    if N == 0:
        raise RuntimeError(
            "未检测到任何棋盘格角点。请检查 config.yaml 的 XX/YY 是否与实际标定板一致、"
            "图片是否清晰完整。"
        )

    # 相机标定，得到标定板在相机坐标系下的位姿
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
    logger_.info(f"相机标定 RMS 重投影误差: {ret:.4f} 像素（成功检测 {N}/{len(images_num)} 张）")

    # 机械臂末端位姿 -> 齐次变换矩阵（眼在手外时求逆），存为 RobotToolPose.csv
    poses_to_csv(file_path, os.path.join(path, "RobotToolPose.csv"), invert=invert)
    tool_pose = np.loadtxt(os.path.join(path, "RobotToolPose.csv"), delimiter=',')

    R_tool = []
    t_tool = []
    for i in range(int(N)):
        R_tool.append(tool_pose[0:3, 4 * i:4 * i + 3])
        t_tool.append(tool_pose[0:3, 4 * i + 3])

    R_handeye, t_handeye = cv2.calibrateHandEye(R_tool, t_tool, rvecs, tvecs, cv2.CALIB_HAND_EYE_TSAI)

    return R_handeye, t_handeye


if __name__ == '__main__':

    ap = argparse.ArgumentParser(description="手眼标定计算（统一版）")
    ap.add_argument("--data", help="指定数据夹名(如 data20260629)，默认使用最新")
    ap.add_argument("--mode", choices=["eye_in_hand", "eye_to_hand"],
                    help="覆盖标定模式(默认读 数据夹mode.txt 或 config.yaml)")
    args = ap.parse_args()

    images_path = resolve_images_path(args.data)
    calib_mode = resolve_mode(images_path, args.mode)
    invert = (calib_mode == "eye_to_hand")
    logger_.info(f"数据夹: {images_path} | 标定模式: {calib_mode}（机械臂位姿{'求逆' if invert else '不求逆'}）")

    rotation_matrix, translation_vector = calibrate(images_path, invert)
    quaternion = R.from_matrix(rotation_matrix).as_quat()

    label = "相机 -> 机械臂末端 (cam2end)" if not invert else "相机 -> 机械臂基座 (cam2base)"
    logger_.info(f"手眼标定结果 [{label}]")
    logger_.info(f"旋转矩阵是:\n {rotation_matrix}")
    logger_.info(f"平移向量是(米):\n {translation_vector}")
    logger_.info(f"四元数是(x,y,z,w):\n {quaternion}")

    # 保存标定结果(R, t, 四元数, 模式)供 validate.py 精度验证使用
    np.savez(os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_eye_result.npz"),
             R=rotation_matrix, t=translation_vector.flatten(),
             quaternion=quaternion, mode=calib_mode)
    logger_.info("标定结果已保存到 hand_eye_result.npz")
