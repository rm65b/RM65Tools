"""
高级物体检测模块 - 支持多种检测方法

支持:
1. ArUco 标记检测
2. 颜色检测
3. 边缘检测
4. 深度分割

用法:
    from vision import AdvancedDetector

    detector = AdvancedDetector(camera)
    detections = detector.detect_aruco(color_image)
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from enum import Enum


class DetectionMethod(Enum):
    """检测方法"""
    ARUCO = "aruco"
    COLOR = "color"
    EDGE = "edge"
    DEPTH = "depth"


@dataclass
class Detection:
    """检测结果"""
    method: DetectionMethod
    center_x: int
    center_y: int
    depth: float
    width: int
    height: int
    confidence: float
    id: Optional[int] = None  # ArUco ID 或颜色标签
    angle: Optional[float] = None  # 旋转角度 (度)
    corners: Optional[List[Tuple[int, int]]] = None  # 角点


class ArUcoDetector:
    """ArUco 标记检测器"""

    # ArUco 字典类型
    DICT_TYPES = {
        '4x4': cv2.aruco.DICT_4X4_50,
        '5x5': cv2.aruco.DICT_5X5_50,
        '6x6': cv2.aruco.DICT_6X6_50,
        '7x7': cv2.aruco.DICT_7X7_50,
        'original': cv2.aruco.DICT_ORIGINAL,
    }

    def __init__(self, dict_type: str = '6x6', marker_size: float = 0.05):
        """
        初始化 ArUco 检测器

        Args:
            dict_type: 字典类型 ('4x4', '5x5', '6x6', '7x7', 'original')
            marker_size: 标记实际尺寸 (米)
        """
        self.dict_type = dict_type
        self.marker_size = marker_size
        self.dictionary = cv2.aruco.getPredefinedDictionary(self.DICT_TYPES[dict_type])
        self.parameters = cv2.aruco.DetectorParameters()

        # 调整检测参数
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.minMarkerDistanceRate = 0.05
        self.parameters.maxErroneousBitsInBorderRate = 0.35

    def detect(self, image: np.ndarray) -> List[Detection]:
        """
        检测 ArUco 标记

        Args:
            image: 输入图像

        Returns:
            检测结果列表
        """
        # 转为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # 检测标记
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        detections = []

        if ids is not None:
            for i, marker_id in enumerate(ids):
                corner = corners[i][0]

                # 计算中心点
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))

                # 计算尺寸
                width = int(np.max(corner[:, 0]) - np.min(corner[:, 0]))
                height = int(np.max(corner[:, 1]) - np.min(corner[:, 1]))

                # 计算旋转角度
                angle = self._calculate_marker_angle(corner)

                detection = Detection(
                    method=DetectionMethod.ARUCO,
                    center_x=center_x,
                    center_y=center_y,
                    depth=0.0,  # 需要外部提供深度信息
                    width=width,
                    height=height,
                    confidence=1.0,
                    id=int(marker_id),
                    angle=angle,
                    corners=[(int(c[0]), int(c[1])) for c in corner]
                )
                detections.append(detection)

        return detections

    def _calculate_marker_angle(self, corner: np.ndarray) -> float:
        """
        计算标记旋转角度

        Args:
            corner: 角点坐标

        Returns:
            角度 (度)
        """
        # 使用第一个角点指向第二个角点的向量
        vector = corner[1] - corner[0]
        angle = np.degrees(np.arctan2(vector[1], vector[0]))
        return angle

    def draw_detection(self, image: np.ndarray,
                      detection: Detection) -> np.ndarray:
        """
        在图像上绘制检测结果

        Args:
            image: 输入图像
            detection: 检测结果

        Returns:
            绘制后的图像
        """
        result = image.copy()

        if detection.method == DetectionMethod.ARUCO and detection.corners:
            # 绘制标记轮廓
            pts = np.array(detection.corners, dtype=np.int32)
            cv2.polylines(result, [pts], True, (0, 255, 0), 2)

            # 绘制中心点
            cv2.circle(result, (detection.center_x, detection.center_y),
                      5, (0, 0, 255), -1)

            # 绘制ID
            if detection.id is not None:
                cv2.putText(result, f"ID:{detection.id}",
                          (detection.center_x + 10, detection.center_y),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 绘制角度
            if detection.angle is not None:
                cv2.putText(result, f"{detection.angle:.1f}°",
                          (detection.center_x + 10, detection.center_y + 25),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return result


class ColorDetector:
    """颜色检测器"""

    def __init__(self, color_lower: Tuple[int, int, int],
                 color_upper: Tuple[int, int, int],
                 min_area: int = 500):
        """
        初始化颜色检测器

        Args:
            color_lower: HSV 下界 (H, S, V)
            color_upper: HSV 上界 (H, S, V)
            min_area: 最小轮廓面积
        """
        self.color_lower = np.array(color_lower)
        self.color_upper = np.array(color_upper)
        self.min_area = min_area

    def detect(self, image: np.ndarray) -> List[Detection]:
        """
        检测颜色区域

        Args:
            image: 输入图像

        Returns:
            检测结果列表
        """
        # 转换到HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 颜色分割
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)

        # 形态学操作
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        detections = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            # 获取边界框
            x, y, w, h = cv2.boundingRect(contour)

            # 计算中心点
            center_x = x + w // 2
            center_y = y + h // 2

            # 计算填充率作为置信度
            fill_rate = area / (w * h)

            detection = Detection(
                method=DetectionMethod.COLOR,
                center_x=center_x,
                center_y=center_y,
                depth=0.0,
                width=w,
                height=h,
                confidence=fill_rate
            )
            detections.append(detection)

        return detections

    def draw_detection(self, image: np.ndarray,
                      detection: Detection) -> np.ndarray:
        """绘制检测结果"""
        result = image.copy()

        x = detection.center_x - detection.width // 2
        y = detection.center_y - detection.height // 2

        cv2.rectangle(result, (x, y),
                     (x + detection.width, y + detection.height),
                     (0, 255, 0), 2)
        cv2.circle(result, (detection.center_x, detection.center_y),
                  5, (0, 0, 255), -1)

        return result


class EdgeDetector:
    """边缘检测器"""

    def __init__(self, canny_threshold1: int = 50,
                 canny_threshold2: int = 150,
                 min_area: int = 1000):
        """
        初始化边缘检测器

        Args:
            canny_threshold1: Canny 下阈值
            canny_threshold2: Canny 上阈值
            min_area: 最小面积
        """
        self.canny_threshold1 = canny_threshold1
        self.canny_threshold2 = canny_threshold2
        self.min_area = min_area

    def detect(self, image: np.ndarray) -> List[Detection]:
        """检测边缘物体"""
        # 转灰度
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 高斯模糊
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny 边缘检测
        edges = cv2.Canny(blurred, self.canny_threshold1, self.canny_threshold2)

        # 查找轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        detections = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            detection = Detection(
                method=DetectionMethod.EDGE,
                center_x=x + w // 2,
                center_y=y + h // 2,
                depth=0.0,
                width=w,
                height=h,
                confidence=area / (w * h)
            )
            detections.append(detection)

        return detections


class DepthSegmentationDetector:
    """深度分割检测器"""

    def __init__(self, depth_min: float = 0.3,
                 depth_max: float = 1.5,
                 depth_diff_threshold: float = 0.05):
        """
        初始化深度分割检测器

        Args:
            depth_min: 有效深度最小值 (米)
            depth_max: 有效深度最大值 (米)
            depth_diff_threshold: 深度差异阈值 (米)
        """
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.depth_diff_threshold = depth_diff_threshold

    def detect(self, color_image: np.ndarray,
               depth_image: np.ndarray,
               depth_scale: float = 0.001) -> List[Detection]:
        """
        基于深度分割检测物体

        Args:
            color_image: RGB图像
            depth_image: 深度图像
            depth_scale: 深度缩放因子

        Returns:
            检测结果列表
        """
        # 转换深度到米
        depth_m = depth_image.astype(np.float32) * depth_scale

        # 创建有效深度掩码
        valid_mask = (depth_m > self.depth_min) & (depth_m < self.depth_max)

        # 简单的区域生长/连通域检测
        # 这里使用简化的方法: 基于深度阈值分割

        detections = []

        # 计算深度直方图,找到主要深度范围
        hist, bins = np.histogram(depth_m[valid_mask], bins=50)

        # 找到峰值
        if len(hist) > 0:
            peak_idx = np.argmax(hist)
            peak_depth = bins[peak_idx]

            # 创建目标深度掩码
            target_mask = valid_mask & \
                         (depth_m > peak_depth - self.depth_diff_threshold) & \
                         (depth_m < peak_depth + self.depth_diff_threshold)

            # 查找连通域
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                target_mask.astype(np.uint8), 8, cv2.CV_32S
            )

            for i in range(1, num_labels):  # 跳过背景
                area = stats[i, cv2.CC_STAT_AREA]
                if area < 500:
                    continue

                x = stats[i, cv2.CC_STAT_LEFT]
                y = stats[i, cv2.CC_STAT_TOP]
                w = stats[i, cv2.CC_STAT_WIDTH]
                h = stats[i, cv2.CC_STAT_HEIGHT]

                # 获取该区域的平均深度
                region_mask = labels == i
                avg_depth = np.mean(depth_m[region_mask & valid_mask])

                detection = Detection(
                    method=DetectionMethod.DEPTH,
                    center_x=int(centroids[i][0]),
                    center_y=int(centroids[i][1]),
                    depth=avg_depth,
                    width=w,
                    height=h,
                    confidence=area / (w * h)
                )
                detections.append(detection)

        return detections


class AdvancedDetector:
    """高级检测器 - 组合多种检测方法"""

    def __init__(self):
        """初始化检测器"""
        self.aruco_detector = ArUcoDetector(dict_type='6x6')
        self.color_detectors = {}
        self.edge_detector = EdgeDetector()
        self.depth_detector = DepthSegmentationDetector()

    def add_color_detector(self, name: str,
                          color_lower: Tuple[int, int, int],
                          color_upper: Tuple[int, int, int]):
        """
        添加颜色检测器

        Args:
            name: 颜色名称
            color_lower: HSV 下界
            color_upper: HSV 上界
        """
        self.color_detectors[name] = ColorDetector(color_lower, color_upper)

    def detect_all(self, color_image: np.ndarray,
                  depth_image: Optional[np.ndarray] = None,
                  depth_scale: float = 0.001) -> Dict[DetectionMethod, List[Detection]]:
        """
        使用所有方法检测

        Args:
            color_image: RGB图像
            depth_image: 深度图像 (可选)
            depth_scale: 深度缩放因子

        Returns:
            各方法的检测结果
        """
        results = {}

        # ArUco 检测
        aruco_results = self.aruco_detector.detect(color_image)
        results[DetectionMethod.ARUCO] = aruco_results

        # 颜色检测
        for name, detector in self.color_detectors.items():
            color_results = detector.detect(color_image)
            results[f"color_{name}"] = color_results

        # 边缘检测
        edge_results = self.edge_detector.detect(color_image)
        results[DetectionMethod.EDGE] = edge_results

        # 深度分割
        if depth_image is not None:
            depth_results = self.depth_detector.detect(
                color_image, depth_image, depth_scale
            )
            results[DetectionMethod.DEPTH] = depth_results

        return results

    def get_detection_with_depth(self, detection: Detection,
                                  depth_image: np.ndarray,
                                  depth_scale: float = 0.001) -> Detection:
        """
        为检测结果添加深度信息

        Args:
            detection: 检测结果
            depth_image: 深度图像
            depth_scale: 深度缩放因子

        Returns:
            更新后的检测结果
        """
        if detection.depth == 0:
            depth = depth_image[detection.center_y, detection.center_x] * depth_scale
            detection.depth = depth

        return detection


# 使用示例
if __name__ == "__main__":
    print("高级物体检测模块")
    print("\n支持的方法:")
    print("  - ArUco: cv2.aruco 标记检测")
    print("  - Color: HSV 颜色分割")
    print("  - Edge: Canny 边缘检测")
    print("  - Depth: 深度图分割")
