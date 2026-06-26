#!/usr/bin/env python3
"""
查询 RM65 末端 + CTAG2F90D 夹爪完整状态

涵盖: 工具端电源 / RM+协议 / 夹爪状态 / 末端设备信息 / 整机错误码
只读, 不发送任何动作指令。
"""

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

IP, PORT = "192.168.1.18", 8080
VOLTAGE_NAME = {0: "0V", 1: "5V", 2: "12V", 3: "24V"}


def section(title):
    print("\n" + "=" * 50)
    print(f"  {title}")
    print("=" * 50)


def dump(d):
    for k, v in d.items():
        print(f"  {k:16}: {v}")


def main():
    arm = RoboticArm(mode=rm_thread_mode_e.RM_SINGLE_MODE_E)
    h = arm.rm_create_robot_arm(IP, PORT, level=2)
    if h.id == -1:
        print(f"❌ 连接失败 {IP}:{PORT}")
        arm.rm_destroy()
        return
    print(f"✅ 连接 id={h.id}  dof={arm.arm_dof}  Gen{arm.robot_controller_version}")

    # 确保 RM+ 协议开启 (其他脚本可能把末端切到 Modbus, 需切回 RM+ 才能读末端设备)
    import time
    arm.rm_set_tool_voltage(3)
    arm.rm_set_rm_plus_mode(115200)
    for _ in range(4):
        time.sleep(1.5)
        r, _ = arm.rm_get_rm_plus_base_info()
        if r == 0:
            print("末端设备已在线 (RM+ 握手成功)")
            break

    # 1. 工具端电源 / RM+ 协议
    section("工具端电源 / RM+ 协议")
    vt = arm.rm_get_tool_voltage()
    vlevel = vt[1] if isinstance(vt, tuple) and len(vt) > 1 else vt
    print(f"  工具端电压档位 : {vlevel} ({VOLTAGE_NAME.get(vlevel, '?')})")
    pm = arm.rm_get_rm_plus_mode()
    print(f"  RM+ 协议模式   : {pm}")

    # 2. 末端设备基础信息 (RM+ 生态, CTAG2F90D 正确接口)
    section("末端设备基础信息 (rm_get_rm_plus_base_info)")
    ret, bi = arm.rm_get_rm_plus_base_info()
    print(f"  返回码: {ret}" + ("  -> 未识别末端设备" if ret != 0 else ""))
    if ret == 0:
        dump(bi)

    # 4. 末端设备实时状态 (RM+ 生态)
    section("末端设备实时状态 (rm_get_rm_plus_state_info)")
    ret, si = arm.rm_get_rm_plus_state_info()
    print(f"  返回码: {ret}" + ("  -> 未识别末端设备" if ret != 0 else ""))
    if ret == 0:
        dump(si)

    # 5. 整机错误码
    section("整机错误码")
    ret = arm.rm_get_current_arm_state()
    err = ret[-1] if isinstance(ret[-1], dict) else ret[1:]
    print(f"  {err}")
    print("  (8193=夹爪异常, 4105=超速, 0=正常)")

    arm.rm_delete_robot_arm()
    arm.rm_destroy()
    print("\n已断开")


if __name__ == "__main__":
    main()
