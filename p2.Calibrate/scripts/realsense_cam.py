# -*- coding: utf-8 -*-
"""
Intel RealSense D435 取流封装：彩色 + 深度对齐、读取出厂内参、快照保存。
"""
import time
import numpy as np
import pyrealsense2 as rs
import cv2

import config as C


class RealSenseCam:
    def __init__(self, serial, resolution=None, fps=None):
        self.serial = serial
        self.w, self.h = resolution or C.CAM_RESOLUTION
        self.fps = fps or C.CAM_FPS
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.profile = None
        self._intr = None

    def start(self):
        self.cfg.enable_device(self.serial)
        self.cfg.enable_stream(rs.stream.color, self.w, self.h, rs.format.bgr8, self.fps)
        self.cfg.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, self.fps)
        self.profile = self.pipe.start(self.cfg)

        # 深度对齐到彩色，保证像素一一对应
        self.align = rs.align(rs.stream.color)

        intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self._intr = {
            "K": np.array([[intr.fx, 0, intr.ppx],
                           [0, intr.fy, intr.ppy],
                           [0, 0, 1]], dtype=np.float64),
            "dist": np.array(intr.coeffs[:5], dtype=np.float64).reshape(1, -1),
            "width": intr.width, "height": intr.height,
            "model": str(intr.model),
            "ppx": intr.ppx, "ppy": intr.ppy, "fx": intr.fx, "fy": intr.fy,
        }
        # 预热几帧，让 AE/AWB 与曝光稳定
        for _ in range(30):
            self.pipe.wait_for_frames()
        return self

    def intrinsics(self):
        if self._intr is None:
            raise RuntimeError("先 start() 再取内参")
        return self._intr

    def grab(self, warmup_frames=5):
        """
        取一帧稳定的彩色图（及对齐深度）。
        warmup_frames: 抓取前丢弃若干帧，消除运动残影/时延。
        返回 (color_bgr, depth_meters_or_None, timestamp_s)。
        """
        for _ in range(warmup_frames):
            self.pipe.wait_for_frames()
        frameset = self.pipe.wait_for_frames()
        frameset = self.align.process(frameset)
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()
        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_frame.get_units() \
            if depth_frame else None
        ts = color_frame.get_timestamp() / 1000.0
        return color, depth, ts

    def stop(self):
        try:
            self.pipe.stop()
        except Exception:
            pass


def save_intrinsics(serial, intr, source="factory"):
    """把内参写盘（供解算与验证读取）。"""
    import json
    data = {
        "serial": serial,
        "source": source,        # 'factory' 或 'recalibrated'
        "K": intr["K"].tolist(),
        "dist": intr["dist"].reshape(-1).tolist(),
        "width": intr["width"], "height": intr["height"],
        "fx": intr["fx"], "fy": intr["fy"], "ppx": intr["ppx"], "ppy": intr["ppy"],
        "model": intr.get("model", ""),
    }
    path = C.intrinsics_path(serial)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    return path


def load_intrinsics(serial):
    import json
    data = json.loads(C.intrinsics_path(serial).read_text(encoding="utf-8"))
    data["K"] = np.array(data["K"], dtype=np.float64)
    data["dist"] = np.array(data["dist"], dtype=np.float64).reshape(1, -1)
    return data
