"""
RM65 机械臂参数配置模块
RM65 Robot Arm Configuration Module

提供机械臂参数加载和关节限位检查功能
"""

import yaml
from pathlib import Path
from typing import Dict, List, Tuple, Optional


class RobotConfig:
    """机械臂配置类"""

    def __init__(self, config_path: Optional[str] = None):
        """
        初始化机械臂配置

        Args:
            config_path: 配置文件路径，默认为 config/rm65.yaml
        """
        if config_path is None:
            # 默认配置文件路径
            project_root = Path(__file__).parent.parent.parent
            config_path = project_root / "config" / "rm65.yaml"

        self.config_path = Path(config_path)
        self._config = None
        self._load_config()

    def _load_config(self):
        """加载 YAML 配置文件"""
        if not self.config_path.exists():
            raise FileNotFoundError(f"配置文件不存在: {self.config_path}")

        with open(self.config_path, 'r', encoding='utf-8') as f:
            self._config = yaml.safe_load(f)

    @property
    def joint_limits(self) -> Dict[str, Dict[str, float]]:
        """获取关节限位 {j1: {min: -180, max: 180}, ...}"""
        return self._config['joint_limits']

    @property
    def joint_names(self) -> List[str]:
        """获取关节名称列表 ['j1', 'j2', ...]"""
        return list(self._config['joint_limits'].keys())

    @property
    def dof(self) -> int:
        """获取自由度数量"""
        return self._config['robot']['dof']

    def get_joint_limit(self, joint: str) -> Tuple[float, float]:
        """
        获取指定关节的限位

        Args:
            joint: 关节名称 (如 'j1', 'j2')

        Returns:
            (min, max) 最小和最大角度 (度)
        """
        limits = self.joint_limits.get(joint)
        if limits is None:
            raise ValueError(f"未知的关节: {joint}")
        return (limits['min'], limits['max'])

    def check_joint_limit(self, joint: str, angle: float) -> bool:
        """
        检查关节角度是否在限位范围内

        Args:
            joint: 关节名称
            angle: 角度 (度)

        Returns:
            True: 在范围内, False: 超出范围
        """
        min_angle, max_angle = self.get_joint_limit(joint)
        return min_angle <= angle <= max_angle

    def clip_joint_angle(self, joint: str, angle: float) -> float:
        """
        将关节角度限制在有效范围内

        Args:
            joint: 关节名称
            angle: 角度 (度)

        Returns:
            限制后的角度
        """
        min_angle, max_angle = self.get_joint_limit(joint)
        return max(min_angle, min(max_angle, angle))

    def check_all_joints(self, angles: Dict[str, float]) -> Tuple[bool, List[str]]:
        """
        检查所有关节角度是否有效

        Args:
            angles: {j1: 10, j2: 20, ...} 关节角度字典

        Returns:
            (is_valid, invalid_joints) (是否全部有效, 无效关节列表)
        """
        invalid = []
        for joint, angle in angles.items():
            if not self.check_joint_limit(joint, angle):
                invalid.append(joint)
        return (len(invalid) == 0, invalid)

    def normalize_angle(self, joint: str, angle: float) -> float:
        """
        将角度规范化到 [-180, 180] 或关节限位范围内

        Args:
            joint: 关节名称
            angle: 原始角度

        Returns:
            规范化后的角度
        """
        min_angle, max_angle = self.get_joint_limit(joint)

        # 先规范化到 [-180, 180]
        normalized = angle % 360
        if normalized > 180:
            normalized -= 360

        # 再限制到关节范围内
        return self.clip_joint_angle(joint, normalized)


# 预定义配置实例
_config_instance: Optional[RobotConfig] = None


def get_robot_config(config_path: Optional[str] = None) -> RobotConfig:
    """获取机械臂配置实例 (单例模式)"""
    global _config_instance
    if _config_instance is None or config_path is not None:
        _config_instance = RobotConfig(config_path)
    return _config_instance


# 便捷函数
def get_joint_limits() -> Dict[str, Dict[str, float]]:
    """获取所有关节限位"""
    return get_robot_config().joint_limits


def get_joint_names() -> List[str]:
    """获取关节名称列表"""
    return get_robot_config().joint_names


def check_limit(joint: str, angle: float) -> bool:
    """检查关节角度是否在限位范围内"""
    return get_robot_config().check_joint_limit(joint, angle)


def clip_angle(joint: str, angle: float) -> float:
    """将关节角度限制在有效范围内"""
    return get_robot_config().clip_joint_angle(joint, angle)


if __name__ == "__main__":
    # 测试代码
    config = get_robot_config()

    print(f"机械臂: {config._config['robot']['name']}")
    print(f"自由度: {config.dof}")
    print(f"\n关节限位:")
    for joint in config.joint_names:
        min_deg, max_deg = config.get_joint_limit(joint)
        print(f"  {joint}: {min_deg:.1f}° ~ {max_deg:.1f}°")

    print("\n限位检查测试:")
    test_cases = [
        ("j1", 0, True),
        ("j1", 200, False),
        ("j2", -130, True),
        ("j2", -140, False),
        ("j6", 360, True),
        ("j6", 400, False),
    ]

    for joint, angle, expected in test_cases:
        result = config.check_joint_limit(joint, angle)
        clipped = config.clip_joint_angle(joint, angle)
        status = "✓" if result == expected else "✗"
        print(f"  {status} {joint}={angle:6.1f}° -> valid={result}, clipped={clipped:.1f}°")
