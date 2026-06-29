#!/usr/bin/env python3
"""
轨迹记录与回放工具

支持:
1. 记录机器人运动轨迹
2. 回放轨迹
3. 轨迹平滑与优化

用法:
    conda激活 rm65
    python test_trajectory.py record   # 记录轨迹
    python test_trajectory.py replay   # 回放轨迹
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'core'))

import json
import time
import numpy as np
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, asdict
from pathlib import Path

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e


@dataclass
class TrajectoryPoint:
    """轨迹点"""
    timestamp: float              # 时间戳
    joint_angles: List[float]      # 关节角度 (6个)
    end_effector_pose: List[float] # 末端位姿 [x, y, z, rx, ry, rz]


@dataclass
class Trajectory:
    """轨迹数据"""
    name: str
    description: str
    created_at: float
    points: List[TrajectoryPoint]
    duration: float               # 总时长 (秒)


class TrajectoryRecorder:
    """轨迹记录器"""

    def __init__(self, arm_ip: str = "192.168.1.18"):
        """初始化记录器"""
        self.arm_ip = arm_ip
        self.arm: Optional[RoboticArm] = None
        self.is_recording = False
        self.trajectory_points: List[TrajectoryPoint] = []
        self.start_time: float = 0
        self.sample_rate = 10  # Hz

    def connect(self) -> bool:
        """连接机械臂"""
        try:
            self.arm = RoboticArm(mode=rm_thread_mode_e.RM_SINGLE_MODE_E)
            handle = self.arm.rm_create_robot_arm(self.arm_ip, 8080, level=2)
            if handle.id == -1:
                print("❌ 机械臂连接失败")
                return False
            print("✅ 机械臂连接成功")
            return True
        except Exception as e:
            print(f"❌ 连接异常: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        if self.arm:
            self.arm.rm_delete_robot_arm()
            self.arm.rm_destroy()

    def start_recording(self, sample_rate: int = 10):
        """
        开始记录

        Args:
            sample_rate: 采样率 (Hz)
        """
        self.is_recording = True
        self.trajectory_points = []
        self.sample_rate = sample_rate
        self.start_time = time.time()
        print(f"✅ 开始记录轨迹 (采样率: {sample_rate} Hz)")

    def stop_recording(self) -> Trajectory:
        """停止记录并返回轨迹"""
        self.is_recording = False
        duration = time.time() - self.start_time

        trajectory = Trajectory(
            name=f"trajectory_{int(time.time())}",
            description="Recorded trajectory",
            created_at=time.time(),
            points=self.trajectory_points,
            duration=duration
        )

        print(f"✅ 记录完成, 共 {len(self.trajectory_points)} 个点, 时长 {duration:.2f}s")
        return trajectory

    def record_point(self):
        """记录一个轨迹点"""
        if not self.is_recording:
            return

        try:
            # 一次获取关节角度(度) + 末端位姿
            ret, state = self.arm.rm_get_current_arm_state()
            if ret != 0:
                return

            point = TrajectoryPoint(
                timestamp=time.time() - self.start_time,
                joint_angles=state.get('joint', []),
                end_effector_pose=state.get('pose', [0, 0, 0, 0, 0, 0])
            )

            self.trajectory_points.append(point)

        except Exception as e:
            print(f"❌ 记录点失败: {e}")

    def record_loop(self, duration: float = 0):
        """
        记录循环

        Args:
            duration: 记录时长 (秒), 0表示手动停止
        """
        print("记录中... 按 Ctrl+C 停止")

        try:
            if duration > 0:
                end_time = time.time() + duration
                while time.time() < end_time:
                    self.record_point()
                    time.sleep(1.0 / self.sample_rate)
            else:
                while True:
                    self.record_point()
                    time.sleep(1.0 / self.sample_rate)

        except KeyboardInterrupt:
            print("\n⚠️  用户中断")


class TrajectoryReplayer:
    """轨迹回放器"""

    def __init__(self, arm_ip: str = "192.168.1.18"):
        """初始化回放器"""
        self.arm_ip = arm_ip
        self.arm: Optional[RoboticArm] = None
        self.trajectory: Optional[Trajectory] = None
        self.is_playing = False

    def connect(self) -> bool:
        """连接机械臂"""
        try:
            self.arm = RoboticArm(mode=rm_thread_mode_e.RM_SINGLE_MODE_E)
            handle = self.arm.rm_create_robot_arm(self.arm_ip, 8080, level=2)
            if handle.id == -1:
                print("❌ 机械臂连接失败")
                return False
            print("✅ 机械臂连接成功")
            return True
        except Exception as e:
            print(f"❌ 连接异常: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        if self.arm:
            self.arm.rm_delete_robot_arm()
            self.arm.rm_destroy()

    def load_trajectory(self, filepath: str) -> bool:
        """加载轨迹文件"""
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)

            # 从字典重建对象
            points_data = data.pop('points')
            self.trajectory = Trajectory(
                points=[TrajectoryPoint(**p) for p in points_data],
                **data
            )

            print(f"✅ 轨迹加载成功:")
            print(f"  名称: {self.trajectory.name}")
            print(f"  点数: {len(self.trajectory.points)}")
            print(f"  时长: {self.trajectory.duration:.2f}s")

            return True

        except Exception as e:
            print(f"❌ 加载轨迹失败: {e}")
            return False

    def play(self, speed: float = 1.0, smooth: bool = True,
             follow: bool = False) -> bool:
        """
        连续轨迹回放 (CANFD 关节透传)

        相比逐点 rm_movej + sleep, CANFD 透传让机械臂连续跟随密集关节目标,
        运动平滑无停顿。

        Args:
            speed: 回放速度倍数 (>1 加速, <1 减速)
            smooth: 是否对关节角做移动平均预平滑 (低跟随时推荐开启)
            follow: True=高跟随(需≤10ms周期, 适合高频插值轨迹),
                    False=低跟随(适合 10Hz 级示教轨迹, 默认)

        Returns:
            bool: 是否成功完成
        """
        if self.trajectory is None:
            print("❌ 请先加载轨迹")
            return False

        points = self.trajectory.points
        if not points:
            print("❌ 轨迹为空")
            return False

        if smooth and len(points) > 3:
            points = self._smooth_trajectory(points)

        mode_name = "高跟随" if follow else "低跟随"
        print(f"开始连续回放: {len(points)} 点 | CANFD {mode_name} | 速度 {speed}x")

        self.is_playing = True
        try:
            # 1) 规划运动到起点, 避免从当前位置突变
            print("→ 规划运动到轨迹起点...")
            ret = self.arm.rm_movej(points[0].joint_angles, 20, 0, 0, 0)
            if ret != 0:
                print(f"❌ 起点规划失败 (code={ret})")
                return False
            self._wait_until_reached(points[0].joint_angles, timeout=15.0)

            # 2) CANFD 连续透传后续关节目标, 按原始时间戳节拍
            print("→ 开始透传轨迹...")
            prev_t = points[0].timestamp
            fail_streak = 0
            for i, p in enumerate(points[1:], 1):
                if not self.is_playing:
                    print("⚠️  回放已停止")
                    break

                ret = self.arm.rm_movej_canfd(p.joint_angles, follow)
                if ret != 0:
                    fail_streak += 1
                    if fail_streak >= 5:
                        print(f"❌ 连续透传失败, 中止 (code={ret}, 点 {i}/{len(points)})")
                        return False
                else:
                    fail_streak = 0

                # 按原始时间戳节拍, 受 speed 缩放
                dt = (p.timestamp - prev_t) / speed
                if dt > 0:
                    time.sleep(dt)
                prev_t = p.timestamp

            if self.is_playing:
                print("✅ 回放完成")
            return True

        except KeyboardInterrupt:
            print("\n⚠️  回放中断")
            self.is_playing = False
            return False

        except Exception as e:
            print(f"❌ 回放异常: {e}")
            return False

    def stop(self):
        """停止回放"""
        self.is_playing = False

    def _wait_until_reached(self, target_joint: List[float],
                            timeout: float = 15.0, tol: float = 1.0) -> bool:
        """
        轮询关节状态, 等待到达目标 (各关节偏差 < tol 度) 或超时

        Args:
            target_joint: 目标关节角度 (度)
            timeout: 超时时间 (秒)
            tol: 各关节允许偏差 (度)

        Returns:
            bool: 是否在超时前到达
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            ret, state = self.arm.rm_get_current_arm_state()
            if ret == 0:
                cur = state.get('joint', [])
                if cur and all(abs(c - t) < tol for c, t in zip(cur, target_joint)):
                    return True
            time.sleep(0.05)
        print("⚠️  到达起点超时, 继续回放")
        return False

    def _smooth_trajectory(self, points: List[TrajectoryPoint],
                          window_size: int = 5) -> List[TrajectoryPoint]:
        """
        平滑轨迹 (移动平均)

        Args:
            points: 原始轨迹点
            window_size: 窗口大小

        Returns:
            平滑后的轨迹点
        """
        smoothed = []

        for i in range(len(points)):
            # 确定窗口范围
            start = max(0, i - window_size // 2)
            end = min(len(points), i + window_size // 2 + 1)

            # 计算平均值
            window = points[start:end]
            avg_joint = np.mean([p.joint_angles for p in window], axis=0).tolist()

            # 创建平滑点
            smoothed_point = TrajectoryPoint(
                timestamp=points[i].timestamp,
                joint_angles=avg_joint,
                end_effector_pose=points[i].end_effector_pose
            )
            smoothed.append(smoothed_point)

        return smoothed


def save_trajectory(trajectory: Trajectory, filepath: str):
    """保存轨迹到文件"""
    data = asdict(trajectory)
    data['points'] = [asdict(p) for p in trajectory.points]

    with open(filepath, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"✅ 轨迹已保存: {filepath}")


def list_trajectories(directory: str):
    """列出可用轨迹"""
    path = Path(directory)
    trajectories = list(path.glob("*.json"))

    if not trajectories:
        print("❌ 没有找到轨迹文件")
        return []

    print("\n可用轨迹:")
    for i, traj_file in enumerate(trajectories, 1):
        try:
            with open(traj_file, 'r') as f:
                data = json.load(f)
            print(f"  {i}. {data.get('name', traj_file.name)}")
            print(f"     文件: {traj_file.name}")
            print(f"     点数: {len(data.get('points', []))}")
            print(f"     时长: {data.get('duration', 0):.2f}s")
        except:
            print(f"  {i}. {traj_file.name} (无法读取)")

    return trajectories


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="轨迹记录与回放工具")
    parser.add_argument('action', choices=['record', 'replay', 'list'],
                       help='操作: record=记录, replay=回放, list=列出')
    parser.add_argument('--file', help='轨迹文件路径')
    parser.add_argument('--duration', type=float, default=0,
                       help='记录时长 (秒), 0=手动停止')
    parser.add_argument('--speed', type=float, default=1.0,
                       help='回放速度倍数')
    parser.add_argument('--rate', type=int, default=10,
                       help='采样率 (Hz)')
    parser.add_argument('--ip', default='192.168.1.18',
                       help='机械臂IP地址')

    args = parser.parse_args()

    output_dir = os.path.join(
        os.path.dirname(__file__), '..', '..', 'tests', 'output', 'trajectories'
    )
    output_dir = os.path.abspath(output_dir)
    os.makedirs(output_dir, exist_ok=True)

    if args.action == 'list':
        list_trajectories(output_dir)
        return

    if args.action == 'record':
        print("""
╔════════════════════════════════════════════════════════════╗
║              轨迹记录模式                                  ║
╠════════════════════════════════════════════════════════════╣
║  操作说明:                                                 ║
║  1. 启动记录后, 手动拖动机械臂或示教器                   ║
║  2. 系统会自动记录关节角度                                 ║
║  3. 按 Ctrl+C 停止记录                                    ║
║  4. 轨迹自动保存到输出目录                                ║
╚════════════════════════════════════════════════════════════╝
        """)

        recorder = TrajectoryRecorder(arm_ip=args.ip)
        if not recorder.connect():
            return

        try:
            recorder.start_recording(sample_rate=args.rate)
            recorder.record_loop(duration=args.duration)
            trajectory = recorder.stop_recording()

            # 保存轨迹
            filename = f"{trajectory.name}.json"
            filepath = os.path.join(output_dir, filename)
            save_trajectory(trajectory, filepath)

        finally:
            recorder.disconnect()

    elif args.action == 'replay':
        if not args.file:
            # 列出轨迹让用户选择
            trajectories = list_trajectories(output_dir)
            if not trajectories:
                return

            try:
                choice = int(input("\n选择轨迹编号: ")) - 1
                if 0 <= choice < len(trajectories):
                    args.file = str(trajectories[choice])
                else:
                    print("❌ 无效选择")
                    return
            except ValueError:
                print("❌ 无效输入")
                return

        print("""
╔════════════════════════════════════════════════════════════╗
║              轨迹回放模式                                  ║
╠════════════════════════════════════════════════════════════╣
║  警告: 回放前请确保:                                       ║
║  - 工作空间无人员                                         ║
║  - 机械臂已上电并使能                                     ║
║  - 有紧急停止准备                                         ║
╚════════════════════════════════════════════════════════════╝
        """)

        input("按 Enter 开始回放...")

        replayer = TrajectoryReplayer(arm_ip=args.ip)
        if not replayer.connect():
            return

        try:
            if not replayer.load_trajectory(args.file):
                return

            replayer.play(speed=args.speed, smooth=True)

        finally:
            replayer.disconnect()


if __name__ == "__main__":
    main()
