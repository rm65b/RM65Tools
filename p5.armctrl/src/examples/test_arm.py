#!/usr/bin/env python3
"""
RM65-B 机械臂测试脚本

用法:
    conda activate rm65
    python test_arm.py
"""

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
import time


def test_connection():
    """测试机械臂连接"""
    print('=== 测试 1: 连接机械臂 ===')
    arm = RoboticArm(mode=rm_thread_mode_e.RM_SINGLE_MODE_E)
    handle = arm.rm_create_robot_arm("192.168.1.18", 8080, level=2)

    if handle.id == -1:
        print('  ❌ 连接失败')
        return None

    print(f'  ✅ 连接成功')
    print(f'     ID: {handle.id}')
    print(f'     自由度: {arm.arm_dof}')
    print(f'     控制器版本: Gen {arm.robot_controller_version}')
    return arm


def test_get_joint_angle(arm):
    """测试读取关节角度"""
    print('\n=== 测试 2: 读取关节角度 ===')
    ret, angles = arm.rm_get_current_joint_current()
    print(f'  当前关节角度: {angles}')
    if ret == 0:
        print('  ✅ 读取成功')
    else:
        print(f'  ⚠️  返回码: {ret}')


def test_get_arm_state(arm):
    """测试读取机械臂状态"""
    print('\n=== 测试 3: 读取机械臂状态 ===')
    ret, state = arm.rm_get_arm_all_state()
    print(f'  返回码: {ret}')
    if ret == 0:
        print('  ✅ 状态读取成功')
        print(f'  运动模式: {state.get("motion_mode", "未知")}')
        print(f'  碰撞检测: {state.get("collision_state", "未知")}')
    else:
        print(f'  ⚠️  状态读取失败')


def show_move_methods(arm):
    """显示可用运动方法"""
    print('\n=== 可用运动方法 ===')
    move_methods = [m for m in dir(arm) if 'move' in m.lower() and not m.startswith('_')]
    print(f'  共 {len(move_methods)} 个方法:')
    for m in move_methods[:10]:
        print(f'    - {m}')


def test_move(arm):
    """测试关节运动（需要安全环境）"""
    print('\n=== 测试 4: 关节运动（可选）===')
    print('  ⚠️  警告: 执行运动前请确保:')
    print('    - 机械臂已上电')
    print('    - 伺服已使能')
    print('    - 工作空间无障碍物')
    print('    - 有人为紧急停止准备')
    print()

    choice = input('是否执行运动测试? (y/N): ').strip().lower()
    if choice != 'y':
        print('  跳过运动测试')
        return

    # 移动到零位（小幅度）
    print('  移动到零位...')
    ret = arm.rm_movej([0, 0, 0, 0, 0, 0], 20, 0, 0, 0)  # v=20, r=0, connect=0(立即执行), block=0(非阻塞)
    if ret == 0:
        print('  ✅ 运动指令发送成功')
        time.sleep(2)
    else:
        print(f'  ❌ 运动失败，错误码: {ret}')


def main():
    """主测试函数"""
    print('=' * 60)
    print('        RM65-B 机械臂测试程序')
    print('=' * 60)

    arm = None
    try:
        # 连接测试
        arm = test_connection()
        if arm is None:
            return

        # 状态测试
        test_get_joint_angle(arm)
        test_get_arm_state(arm)

        # 显示可用方法
        show_move_methods(arm)

        # 运动测试（可选）
        test_move(arm)

        print('\n' + '=' * 60)
        print('  测试完成!')
        print('=' * 60)

    except Exception as e:
        print(f'\n❌ 测试失败: {e}')
        import traceback
        traceback.print_exc()

    finally:
        # 断开连接
        if arm is not None:
            print('\n断开连接...')
            arm.rm_delete_robot_arm()
            arm.rm_destroy()
            print('  ✅ 已断开')


if __name__ == "__main__":
    main()
