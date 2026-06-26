#!/usr/bin/env python3
"""
获取 RM65 机械臂实时状态

用法:
    conda activate rm65
    python get_arm_state.py [--ip 192.168.1.18] [--port 8080]

读取内容: 连接信息 / 关节角度 / 末端位姿 / 整机状态 / 夹爪状态
仅读取, 不发送任何运动指令, 安全。
"""

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
import argparse


def safe_call(arm, name):
    """安全调用某个状态读取方法, 失败不影响其他调用"""
    fn = getattr(arm, name, None)
    if fn is None:
        return  # 该 SDK 版本无此方法
    try:
        ret = fn()
    except Exception as e:
        print(f"  [{name}] 调用异常: {e}")
        return
    # RealMan API 通常返回 (code, data...) 或 (code, data)
    if isinstance(ret, tuple) and len(ret) >= 1:
        code = ret[0]
        data = ret[1:] if len(ret) > 1 else (None,)
        print(f"  [{name}] 返回码={code}")
        for d in data:
            if isinstance(d, dict):
                for k, v in d.items():
                    print(f"      {k}: {v}")
            else:
                print(f"      {d}")
    else:
        print(f"  [{name}] -> {ret}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.1.18", help="机械臂IP")
    parser.add_argument("--port", type=int, default=8080, help="机械臂端口")
    args = parser.parse_args()

    print("=" * 55)
    print("  RM65 机械臂状态获取 (只读)")
    print("=" * 55)

    # 1. 连接
    arm = RoboticArm(mode=rm_thread_mode_e.RM_SINGLE_MODE_E)
    handle = arm.rm_create_robot_arm(args.ip, args.port, level=2)
    if handle.id == -1:
        print(f"❌ 连接失败 {args.ip}:{args.port} (handle.id=-1)")
        arm.rm_destroy()
        return
    print(f"✅ 连接成功  id={handle.id}  dof={arm.arm_dof}  "
          f"控制器Gen={arm.robot_controller_version}")

    # 2. 列出 SDK 提供的状态读取方法
    state_methods = sorted(m for m in dir(arm)
                           if m.startswith(("rm_get_", "rm_read_")))
    print(f"\n[SDK] 可用状态读取方法 ({len(state_methods)} 个)")

    # 3. 调用关键状态接口
    print("\n--- 关节角度 ---")
    safe_call(arm, "rm_get_current_joint_angle")   # 当前关节角(度)
    safe_call(arm, "rm_get_joint_angle")           # 目标关节角

    print("\n--- 末端位姿 ---")
    safe_call(arm, "rm_get_current_arm_state")      # 当前末端位姿 + 关节
    safe_call(arm, "rm_get_tcp")                    # TCP 位姿(若有)

    print("\n--- 整机状态 ---")
    safe_call(arm, "rm_get_arm_all_state")          # 运动模式/碰撞/系统状态

    # 注: 夹爪(CTAG2F90D)状态读取见 get_gripper_status.py (走 RM+ 接口 rm_get_rm_plus_*)

    # 4. 断开
    print("\n" + "=" * 55)
    arm.rm_delete_robot_arm()
    arm.rm_destroy()
    print("✅ 已断开连接")


if __name__ == "__main__":
    main()
