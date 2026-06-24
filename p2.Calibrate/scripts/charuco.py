# -*- coding: utf-8 -*-
"""
CharUco 标定板检测与位姿估计（OpenCV 4.13 新 aruco API）。

坐标系：标定板系原点在棋盘左上角内角点，Z 出板面（ArUco/CharUco 默认）。
solvePnP 返回的 T_cam_board = “板在相机系下”的位姿。
"""
import numpy as np
import cv2
import cv2.aruco as aruco
import config as C


def get_dict():
    """按 config.ARUCO_DICT 名取预定义字典。"""
    if C.BOARD_DICT_ID is not None:
        return aruco.getPredefinedDictionary(int(C.BOARD_DICT_ID))
    name = C.ARUCO_DICT
    if name not in C.ARUCO_DICTS:
        raise ValueError(f"未知字典 {name}，可选: {list(C.ARUCO_DICTS)}")
    return aruco.getPredefinedDictionary(C.ARUCO_DICTS[name])


def make_board():
    """
    构造 CharUco 板对象（新版构造器，无 _create 后缀）。
    返回 (board, detector)。
    """
    dictionary = get_dict()
    board = aruco.CharucoBoard(
        (C.BOARD_SQUARES_X, C.BOARD_SQUARES_Y),
        C.BOARD_SQUARE_LEN_M,
        C.BOARD_MARKER_LEN_M,
        dictionary,
    )
    detector = aruco.CharucoDetector(board)
    return board, detector


def board_objpoints(board, charuco_ids):
    """
    把检测到的 charuco id 映射到板对象坐标（3D，米）。
    board.getChessboardCorners() 返回全部棋盘内角点 (N,3)。
    """
    obj = np.asarray(board.getChessboardCorners(), dtype=np.float64)
    ids = np.asarray(charuco_ids).reshape(-1)
    return obj[ids]


def detect(detector, image):
    """
    检测 CharUco 角点。
    返回 (charuco_corners, charuco_ids) 或 (None, None)（检测失败/角点太少）。
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    charuco_corners, charuco_ids, _mc, _mi = detector.detectBoard(gray)
    if charuco_corners is None or len(charuco_corners) < C.MIN_CHARUCO_CORNERS:
        return None, None
    return charuco_corners.reshape(-1, 2), np.asarray(charuco_ids).reshape(-1)


def estimate_pose(board, charuco_corners, charuco_ids, intrinsics):
    """
    solvePnP → 4x4 T_cam_board（板在相机系下）。
    intrinsics: dict with keys K(3x3), dist(1x5).
    """
    obj_pts = board_objpoints(board, charuco_ids).reshape(-1, 1, 3)
    img_pts = charuco_corners.reshape(-1, 1, 2)
    ok, rvec, tvec = cv2.solvePnP(
        obj_pts, img_pts, intrinsics["K"], intrinsics["dist"],
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(-1)
    return T


def draw_detected(image, charuco_corners, charuco_ids):
    """可视化：画角点与 id。"""
    vis = image.copy()
    if charuco_corners is not None:
        cv2.aruco.drawDetectedCornersCharuco(vis, charuco_corners.reshape(-1, 1, 2), charuco_ids)
    return vis


def draw_axis(image, T_cam_board, intrinsics, scale=0.04):
    """在图像上画板系坐标轴。"""
    rvec, _ = cv2.Rodrigues(T_cam_board[:3, :3])
    tvec = T_cam_board[:3, 3].reshape(3, 1)
    return cv2.drawFrameAxes(image, intrinsics["K"], intrinsics["dist"], rvec, tvec, scale)
