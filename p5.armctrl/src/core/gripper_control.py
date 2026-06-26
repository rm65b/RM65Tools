#!/usr/bin/env python3
"""
CTAG2F90D 夹爪控制 (JSON 协议 + Modbus-RTU 寄存器)

用法:
    python gripper_control.py open           # 张开 (位置1000)
    python gripper_control.py close          # 夹紧 (位置0)
    python gripper_control.py position 500   # 自定义位置 (0=夹紧 ~ 1000=张开)
    python gripper_control.py status         # 读取位置/力矩到达状态

原理:
    Gen3 Python API 不支持 Modbus 寄存器读写 (rm_write_modbus_rtu_registers 返回 -4),
    故通过 TCP socket 向 192.168.1.18:8080 发送 JSON 命令控制夹爪。
    寄存器(知行手册): 位置@258(32位) / 触发@264 / 力矩@284

位置极性(CTAG2F90D 实测, 与知行手册 CTAG2F120 标注相反):
    1000 = 张开, 0 = 夹紧
"""

import socket
import json
import sys
import time

IP, PORT = "192.168.1.18", 8080
POS_OPEN = 1000   # 张开
POS_CLOSE = 0     # 夹紧


def jsend(cmd, timeout=4):
    """发送一条 JSON 命令到控制器, 返回响应 dict"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((IP, PORT))
        s.sendall((json.dumps(cmd) + "\n").encode())
        resp = s.recv(4096).decode(errors="ignore").strip()
    finally:
        s.close()
    try:
        return json.loads(resp)
    except Exception:
        return {"raw": resp}


def _pos_to_data(pos):
    """位置值(0~1000) -> 4字节 [258高,258低,259高,259低] (32位大端, 高16位=0)
    例: 1000=0x3E8 -> [0,0,3,232]"""
    pos = max(0, min(1000, int(pos)))
    return [0, 0, (pos >> 8) & 0xFF, pos & 0xFF]


def _ensure_modbus():
    """确保工具端 24V + Modbus-RTU 主站"""
    jsend({"command": "set_tool_voltage", "voltage_type": 3})
    time.sleep(0.3)
    jsend({"command": "set_modbus_mode", "port": 1, "baudrate": 115200, "timeout": 2})
    time.sleep(1.0)


def set_position(position, torque=50):
    """设置夹爪位置 (0=夹紧 ~ 1000=张开) 并触发运动"""
    _ensure_modbus()
    jsend({"command": "write_single_register", "port": 1, "address": 284,
           "data": torque, "device": 1})
    time.sleep(0.3)
    data = _pos_to_data(position)
    r = jsend({"command": "write_registers", "port": 1, "address": 258,
               "num": 2, "data": data, "device": 1})
    print(f"  写位置 {position} (data={data}): {_ok(r)}")
    time.sleep(0.3)
    r = jsend({"command": "write_single_register", "port": 1, "address": 264,
               "data": 1, "device": 1})
    print(f"  触发运行: {_ok(r)}")
    time.sleep(2.0)


def open_gripper():
    print("=== 张开夹爪 (位置1000) ===")
    set_position(POS_OPEN)


def close_gripper():
    print("=== 夹紧夹爪 (位置0) ===")
    set_position(POS_CLOSE)


def status():
    _ensure_modbus()
    pos_reach = jsend({"command": "read_holding_registers", "port": 1,
                       "address": 1538, "device": 1})
    force_reach = jsend({"command": "read_holding_registers", "port": 1,
                         "address": 1537, "device": 1})
    print(f"位置到达@1538: {pos_reach}")
    print(f"力矩到达@1537: {force_reach}")


def _ok(r):
    return "✅" if (r.get("write_state") or r.get("set_state") or r.get("state")) else "⚠️"


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    cmd = sys.argv[1].lower()
    if cmd == "open":
        open_gripper()
    elif cmd == "close":
        close_gripper()
    elif cmd == "position" and len(sys.argv) >= 3:
        set_position(int(sys.argv[2]))
    elif cmd == "status":
        status()
    else:
        print(f"未知命令: {cmd}")
        print("用法: python gripper_control.py [open|close|position N|status]")
        sys.exit(1)
    print("完成")


if __name__ == "__main__":
    main()
