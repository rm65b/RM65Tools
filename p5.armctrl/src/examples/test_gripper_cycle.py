#!/usr/bin/env python3
"""
CTAG2F90D 夹爪循环/耐久测试

用法:
    python gripper_cycle_test.py [循环次数] [间隔秒数]
    python gripper_cycle_test.py 100 2     # 100 次开合, 每动作间隔 2 秒 (默认)

每循环: 张开(1000) -> 间隔 -> 闭合(0) -> 间隔
- 初始化只做一次 (24V + Modbus-RTU + 力矩), 避免每轮重复
- Ctrl+C 可中断, 会打印已完成次数
"""

import socket
import json
import time
import sys

IP, PORT = "192.168.1.18", 8080


def jsend(cmd, timeout=5):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((IP, PORT))
        s.sendall((json.dumps(cmd) + "\n").encode())
        resp = s.recv(4096).decode(errors="ignore").strip()
        try:
            return json.loads(resp)
        except Exception:
            return {"raw": resp}
    finally:
        s.close()


def move(position):
    """位置 1000=张开, 0=闭合"""
    data = [0, 0, (position >> 8) & 0xFF, position & 0xFF]
    jsend({"command": "write_registers", "port": 1, "address": 258,
           "num": 2, "data": data, "device": 1})
    jsend({"command": "write_single_register", "port": 1, "address": 264,
           "data": 1, "device": 1})


def main():
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    print(f"=== 夹爪循环测试: {N} 次开合, 每动作间隔 {interval}s ===", flush=True)

    # 初始化一次
    jsend({"command": "set_tool_voltage", "voltage_type": 3}); time.sleep(2.0)
    jsend({"command": "set_modbus_mode", "port": 1, "baudrate": 115200, "timeout": 2}); time.sleep(1.0)
    jsend({"command": "write_single_register", "port": 1, "address": 284, "data": 50, "device": 1})
    print("初始化完成 (24V + Modbus-RTU + 力矩50)", flush=True)

    done = 0
    fail = 0
    t0 = time.time()
    try:
        for i in range(1, N + 1):
            try:
                move(1000)                     # 张开
                print(f"[{i}/{N}] 张开  (t+{time.time()-t0:.0f}s)", flush=True)
                time.sleep(interval)
                move(0)                        # 闭合
                print(f"[{i}/{N}] 闭合  (t+{time.time()-t0:.0f}s)", flush=True)
                time.sleep(interval)
                done += 1
            except Exception as e:
                fail += 1
                print(f"[{i}/{N}] ⚠️ 异常: {e}", flush=True)
                time.sleep(1)
    except KeyboardInterrupt:
        print(f"\n⚠️ 用户中断, 已完成 {done}/{N} 次循环 (失败 {fail})", flush=True)
        return

    print(f"\n✅ 完成 {done}/{N} 次循环, 失败 {fail}, 总用时 {time.time()-t0:.0f}s", flush=True)


if __name__ == "__main__":
    main()
