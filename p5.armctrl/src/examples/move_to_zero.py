#!/usr/bin/env python3
"""
RM65-B 机械臂归零

用法:
    conda activate rm65
    python move_to_zero.py                          # 归零 (默认速度 20%)
    python move_to_zero.py --speed 30               # 指定关节速度
    python move_to_zero.py --ip 192.168.1.18 --port 8080

原理:
    rm_movej([0,0,0,0,0,0], speed, 0, 0, 1)
        关节空间运动到零位, 平滑=0, 立即执行(0), 阻塞到位(1)
    零位: 6 关节全为 0°, 末端竖直向上 (法兰 TCP 高度 z≈0.85m)

⚠ 执行前务必确认:
    - 机械臂已上电、伺服已使能
    - 工作空间无人员/障碍物
    - 有人为紧急停止准备
"""

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
import argparse
import time

ZERO_JOINTS = [0, 0, 0, 0, 0, 0]


def main():
    parser = argparse.ArgumentParser(description="RM65-B 机械臂归零")
    parser.add_argument("--ip", default="192.168.1.18", help="机械臂IP")
    parser.add_argument("--port", type=int, default=8080, help="机械臂端口")
    parser.add_argument("--speed", type=int, default=20,
                        help="关节速度 1-100 (百分比), 默认 20")
    args = parser.parse_args()

    print("=" * 50)
    print("  RM65-B 机械臂归零")
    print("=" * 50)

    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    try:
        h = arm.rm_create_robot_arm(args.ip, args.port)
        if getattr(h, "id", -1) == -1:
            print(f"❌ 连接失败 {args.ip}:{args.port}")
            return

        # 归零前状态
        code, before = arm.rm_get_current_arm_state()
        if code != 0:
            print(f"❌ 读取状态失败 code={code}（0=成功）")
            return
        print("归零前关节(°):", [round(j, 2) for j in before["joint"]],
              " err:", before.get("err"))

        # 执行归零 (阻塞到位)
        print(f"\n执行归零 (速度 {args.speed}%) ...")
        ret = arm.rm_movej(ZERO_JOINTS, args.speed, 0, 0, 1)
        print(f"rm_movej 返回码: {ret}  (0=成功)")
        if ret != 0:
            print("❌ 归零失败")
            return

        # 归零后确认
        time.sleep(0.5)
        _, after = arm.rm_get_current_arm_state()
        print("归零后关节(°):", [round(j, 2) for j in after["joint"]])
        print("归零后位姿  :", [round(v, 4) for v in after["pose"]],
              "  err:", after.get("err"))
        print("\n✅ 归零完成")

    finally:
        try:
            arm.rm_delete_robot_arm()
        except Exception:
            pass


if __name__ == "__main__":
    main()
