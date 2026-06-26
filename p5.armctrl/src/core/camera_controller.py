"""
RealSense D435 相机控制器
"""

import time

import pyrealsense2 as rs
import numpy as np
import cv2
from typing import Tuple, Optional, Dict


class RealSenseD435:
    """Intel RealSense D435 深度相机控制器"""

    def __init__(self,
                 width: int = 640,
                 height: int = 480,
                 fps: int = 30,
                 serial: Optional[str] = None):
        """
        初始化D435相机

        Args:
            width: 图像宽度
            height: 图像高度
            fps: 帧率
            serial: 指定相机序列号；None=取第一个可用相机（多台时顺序不定，
                    建议钉死序列号以确保抓到目标相机）
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.serial = serial

        self.pipeline: Optional[rs.pipeline] = None
        self.config: Optional[rs.config] = None
        self.is_running = False

        # 深度相机参数
        self.depth_scale = 0.0
        self.intrinsics: Optional[rs.intrinsics] = None

    def connect(self) -> bool:
        """
        连接相机并验证出帧。

        规避部分 D435 单元"重开 pipeline 卡死、无帧"的毛病（直接重开会
        Frame didn't arrive，但 hardware_reset 后稳定）：pipeline 启动后
        先探一帧；若无帧，则对该相机做一次 hardware_reset 并重连。
        正常相机首帧秒到，零额外开销；仅卡死的那台会触发 ~8s 复位。

        Returns:
            bool: 连接成功且能出帧才返回 True
        """
        max_attempts = 2  # 首次连接 + 复位重连一次
        for attempt in range(1, max_attempts + 1):
            try:
                # 创建管道
                self.pipeline = rs.pipeline()
                self.config = rs.config()

                # 指定相机序列号（多台时钉死某一台，避免抓到不确定的那台）
                if self.serial:
                    self.config.enable_device(self.serial)

                # 配置流
                self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
                self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

                # 启动管道
                profile = self.pipeline.start(self.config)

                # 深度参数 + 内参
                self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
                color_stream = profile.get_stream(rs.stream.color)
                self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
                self.is_running = True

                # 探一帧，确认传感器真的在出帧（检测"重开卡死"）
                if self._wait_first_frame(timeout_ms=5000):
                    try:
                        dev_sn = profile.get_device().get_info(rs.camera_info.serial_number)
                    except Exception:
                        dev_sn = self.serial or "auto"
                    print(f"✓ D435相机连接成功 (序列号: {dev_sn})")
                    print(f"  分辨率: {self.width}x{self.height} @ {self.fps}fps")
                    print(f"  深度单位: {self.depth_scale} 米")
                    print(f"  内参: fx={self.intrinsics.fx:.2f}, fy={self.intrinsics.fy:.2f}")
                    return True

                # 无帧：拆管道，复位重连
                print(f"⚠ 相机启动但无帧（疑似重开卡死），第{attempt}/{max_attempts}次：hardware_reset 后重连...")
                self._stop_pipeline()
                if attempt < max_attempts and self._hardware_reset_target():
                    time.sleep(8)  # 等待重新枚举
                    continue
                break  # 复位失败或次数用尽

            except Exception as e:
                print(f"✗ 相机连接失败: {e}")
                self._stop_pipeline()
                return False

        self.is_running = False
        print("✗ 连接后仍无帧（相机可能硬件故障或线缆问题）")
        return False

    def _wait_first_frame(self, timeout_ms: int = 5000) -> bool:
        """启动后探一帧，确认传感器出帧。超时返回 False（疑似卡死）。"""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms)
            return bool(frames.get_color_frame()) or bool(frames.get_depth_frame())
        except Exception:
            return False

    def _hardware_reset_target(self) -> bool:
        """对目标序列号的相机做一次 USB 硬件复位。无序列号时返回 False。"""
        if not self.serial:
            return False
        try:
            for d in rs.context().query_devices():
                if d.get_info(rs.camera_info.serial_number) == self.serial:
                    d.hardware_reset()
                    return True
            print(f"  未找到序列号 {self.serial}，无法复位")
        except Exception as e:
            print(f"  hardware_reset 失败: {e}")
        return False

    def _stop_pipeline(self):
        """安全停止并清理当前管道。"""
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.pipeline = None
        self.config = None
        self.is_running = False

    def disconnect(self):
        """断开连接"""
        if self.pipeline:
            self.pipeline.stop()
            self.is_running = False
            print("✓ 相机已断开")

    def get_frame(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        获取一帧RGB-D数据

        Returns:
            (color_image, depth_image): RGB图像和深度图像
        """
        if not self.is_running:
            return None, None

        try:
            # 等待新帧
            frames = self.pipeline.wait_for_frames()

            # 获取彩色和深度帧
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return None, None

            # 转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            return color_image, depth_image

        except Exception as e:
            print(f"✗ 获取帧失败: {e}")
            return None, None

    def get_depth_at_pixel(self, depth_image: np.ndarray, x: int, y: int) -> float:
        """
        获取指定像素的深度值

        Args:
            depth_image: 深度图像
            x: 像素x坐标
            y: 像素y坐标

        Returns:
            float: 深度值(米)
        """
        if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
            depth = depth_image[y, x] * self.depth_scale
            return depth
        return 0.0

    def deproject_pixel(self, x: int, y: int, depth: float) -> Tuple[float, float, float]:
        """
        像素坐标反投影到3D空间

        Args:
            x: 像素x坐标
            y: 像素y坐标
            depth: 深度值(米)

        Returns:
            (x, y, z): 3D坐标(相机坐标系)
        """
        if not self.intrinsics:
            return 0.0, 0.0, 0.0

        # 使用rs::intrinsics::deproject
        result = rs.rs2_deproject_pixel_to_point(self.intrinsics, [x, y], depth)
        return result[0], result[1], result[2]

    def get_point_cloud(self, color_image: np.ndarray, depth_image: np.ndarray) -> np.ndarray:
        """
        获取点云

        Args:
            color_image: RGB图像
            depth_image: 深度图像

        Returns:
            点云数组 (N, 6) [x, y, z, r, g, b]
        """
        if not self.intrinsics:
            return np.array([])

        points = []
        h, w = depth_image.shape

        for y in range(0, h, 5):  # 降采样以加速
            for x in range(0, w, 5):
                depth = depth_image[y, x] * self.depth_scale
                if 0.3 < depth < 3.0:  # 有效深度范围
                    x_3d, y_3d, z_3d = self.deproject_pixel(x, y, depth)
                    b, g, r = color_image[y, x]
                    points.append([x_3d, y_3d, z_3d, r, g, b])

        return np.array(points)

    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect()


# 使用示例
if __name__ == "__main__":
    with RealSenseD435(width=640, height=480, fps=30) as camera:
        # 获取一帧
        color, depth = camera.get_frame()

        if color is not None:
            print(f"✓ 获取图像成功: {color.shape}")

            # 获取中心点深度
            h, w = color.shape[:2]
            center_depth = camera.get_depth_at_pixel(depth, w//2, h//2)
            print(f"中心点深度: {center_depth:.3f} 米")

            # 计算中心点3D坐标
            x, y, z = camera.deproject_pixel(w//2, h//2, center_depth)
            print(f"中心点3D坐标: ({x:.3f}, {y:.3f}, {z:.3f}) 米")
