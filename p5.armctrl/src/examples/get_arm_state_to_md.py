# -*- coding: utf-8 -*-
"""
连接 RM65 机械臂（RealMan 官方 Python SDK `Robotic_Arm` / RM_API2），
读取末端（法兰 TCP）位姿与各关节信息，输出到 stdout 并写一份 markdown 快照。

单位（取自 SDK 源码 rm_ctypes_wrap.py，权威）：
  pose        = [x, y, z, rx, ry, rz]   x/y/z 单位 m（米），rx/ry/rz 单位 rad（弧度）
  joint       = 各关节角度，单位 °（度）
  温度 ℃ / 电流 mA / 电压 V

用法：
  conda activate rm65
  python get_arm_info.py [ARM_IP] [ARM_PORT]
"""
import sys
import os
import math
import json
from datetime import datetime

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

ARM_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.18"
ARM_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8080
# 项目根目录：src/examples -> src -> 项目根 (p5.armctrl)
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT_DIR = os.path.join(_PROJECT_ROOT, "tests", "output")
OUT_MD = os.path.join(OUT_DIR, "arm_info.md")

# RM65-B MDH 参数（改进 D-H），来源：睿尔曼官方本体参数页
#   https://develop.realman-robotics.com/robot4th/robotParameter/RM65OntologyParameters/
MDH_TABLE = [
    # joint, a(i-1) mm, alpha(i-1) deg, d(i) mm, theta offset deg
    (1, 0,   0,    240.5, 0),
    (2, 0,   90,   0,     90),
    (3, 256, 0,    0,     90),
    (4, 0,   90,   210,   0),
    (5, 0,   -90,  0,     0),
    (6, 0,   90,   "d6",  0),
]
MDH_REF_URL = "https://develop.realman-robotics.com/robot4th/robotParameter/RM65OntologyParameters/"


def rad2deg(x):
    return x * 180.0 / math.pi


def try_call(fn, *args, **kw):
    """安全调用一个可选的 SDK 方法，失败返回 (None, None)。"""
    try:
        return fn(*args, **kw)
    except Exception as e:
        return None, f"<不可用: {e}>"


def main():
    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = arm.rm_create_robot_arm(ARM_IP, ARM_PORT)

    arm_id = getattr(handle, "id", None)
    if arm_id is None:
        try:
            arm_id = handle.contents.id
        except Exception:
            arm_id = None

    # 兜底：用一次状态读取确认链路
    code_state, state = arm.rm_get_current_arm_state()
    if (arm_id is None or arm_id == -1) and code_state != 0:
        print(f"[ERROR] 连接失败 ip={ARM_IP}:{ARM_PORT} handle.id={arm_id} state_code={code_state}",
              file=sys.stderr)
        sys.exit(1)

    arm_dof = getattr(arm, "arm_dof", None) or len(state.get("joint", []) or []) or 6
    arm_type = getattr(arm, "arm_type", None)
    print(f"[arm] 已连接 @ {ARM_IP}:{ARM_PORT} (handle.id={arm_id}, dof={arm_dof}, arm_type={arm_type})")
    print(f"[arm] state code = {code_state}")

    if code_state != 0:
        print(f"[ERROR] rm_get_current_arm_state 失败 code={code_state}", file=sys.stderr)
        sys.exit(1)

    pose = state["pose"]            # [x,y,z,rx,ry,rz] : x/y/z m, rx/ry/rz rad
    joints = list(state["joint"])   # °

    x, y, z = pose[0], pose[1], pose[2]
    rx, ry, rz = pose[3], pose[4], pose[5]

    # 各关节补充信息：温度 / 电流 / 电压
    _, temps = try_call(arm.rm_get_current_joint_temperature)
    _, currs = try_call(arm.rm_get_current_joint_current)
    _, volts = try_call(arm.rm_get_current_joint_voltage)

    # 软件版本（可选，方法名因 SDK 版本而异，按存在性探测）
    swver = None
    for mname in ("rm_get_tool_software_version", "rm_get_software_version"):
        m = getattr(arm, mname, None)
        if callable(m):
            _, swver = try_call(m)
            if swver:
                break

    # ---------------- 控制台输出 ----------------
    print("\n=== 末端位姿 (法兰 TCP, 基坐标系) ===")
    print(f"  位置  x={x:.6f} m  y={y:.6f} m  z={z:.6f} m")
    print(f"        x={x*1000:.3f} mm  y={y*1000:.3f} mm  z={z*1000:.3f} mm")
    print(f"  姿态  rx={rx:.6f} rad ({rad2deg(rx):+.3f}°)  "
          f"ry={ry:.6f} rad ({rad2deg(ry):+.3f}°)  rz={rz:.6f} rad ({rad2deg(rz):+.3f}°)")
    print("\n=== 各关节角度 (°) ===")
    for i, j in enumerate(joints, 1):
        print(f"  J{i} = {j:+.4f}°")

    print("\n=== 各关节状态 ===")
    print(f"  温度(℃) : {(['%.2f' % t for t in temps] if isinstance(temps, list) else temps)}")
    print(f"  电流(mA): {(['%.2f' % c for c in currs] if isinstance(currs, list) else currs)}")
    print(f"  电压(V) : {(['%.2f' % v for v in volts] if isinstance(volts, list) else volts)}")

    # ---------------- 写 markdown ----------------
    def fmt_list(lst, prec=3):
        if isinstance(lst, list):
            return ", ".join(f"{v:.{prec}f}" for v in lst)
        return str(lst)

    lines = []
    lines.append(f"# RM65 机械臂状态快照\n")
    lines.append(f"- 采集时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"- 控制器 IP: `{ARM_IP}:{ARM_PORT}` (USB 网卡直连)")
    lines.append(f"- 连接句柄 id: `{arm_id}`")
    lines.append(f"- 自由度 (DOF): `{arm_dof}`，型号标识 `arm_type={arm_type}`\n")

    # ---- 基坐标系定义（参考基准）----
    lines.append("## 基坐标系定义（参考基准）\n")
    lines.append("本文件所有位姿均相对 RM65 的**基坐标系（base frame，MDH frame {0}）**：\n")
    lines.append("| 要素 | 定义 |")
    lines.append("|---|---|")
    lines.append("| 原点 (0,0,0) | 底座安装法兰面（底面）中心；底座尺寸 φ107 mm，Z=0 落在该安装底面 |")
    lines.append("| Z 轴 | 垂直安装底面向上（沿关节1旋转轴） |")
    lines.append("| X 轴 | J1=0 时指向机械臂正前方（关节2伸出方向） |")
    lines.append("| Y 轴 | 右手定则 |\n")
    lines.append(f"MDH 参数（改进 D-H，来源：[睿尔曼官方本体参数页]({MDH_REF_URL})）：\n")
    lines.append("| joint | a(i-1) mm | α(i-1) ° | d(i) mm | θ offset ° |")
    lines.append("|---|---|---|---|---|")
    for r in MDH_TABLE:
        lines.append("| " + " | ".join(str(c) for c in r) + " |")
    lines.append("")
    lines.append("- `d1 = 240.5 mm`：底座安装法兰面 → 肩部（关节1/2 轴线交汇处）的高度。")
    lines.append("- `d6`（第6轴末端法兰长度）：RM65-B = **144 mm**（本机，见 `config.py` FLANGE_D6_MM）；"
                 "RM65-6F=172.5 / RM65-6FB=161.2 / RM65-B-V=166.8 / RM65-6FB-V=184 mm。")
    lines.append("- 末端 TCP 默认原点落在第6轴法兰面中心；接夹爪/标定板需另设工具坐标系偏移。\n")

    lines.append("## 末端位姿（法兰 TCP，基坐标系）\n")
    lines.append("| 量 | X | Y | Z |")
    lines.append("|---|---|---|---|")
    lines.append(f"| 位置 (m)  | {x:.6f} | {y:.6f} | {z:.6f} |")
    lines.append(f"| 位置 (mm) | {x*1000:.3f} | {y*1000:.3f} | {z*1000:.3f} |\n")
    lines.append("| 量 | rx | ry | rz |")
    lines.append("|---|---|---|---|")
    lines.append(f"| 姿态 (rad) | {rx:.6f} | {ry:.6f} | {rz:.6f} |")
    lines.append(f"| 姿态 (°)   | {rad2deg(rx):+.3f} | {rad2deg(ry):+.3f} | {rad2deg(rz):+.3f} |\n")
    lines.append("> 位姿来自 `rm_get_current_arm_state()`：xyz 单位 **米**，rx/ry/rz 单位 **弧度**（SDK 源码 `rm_position_t`/`rm_euler_t`）。\n")

    lines.append("## 各关节信息\n")
    lines.append("| 关节 | 角度 (°) | 温度 (℃) | 电流 (mA) | 电压 (V) |")
    lines.append("|---|---|---|---|---|")
    for i in range(arm_dof):
        ang = joints[i] if i < len(joints) else float("nan")
        t = temps[i] if isinstance(temps, list) and i < len(temps) else "—"
        c = currs[i] if isinstance(currs, list) and i < len(currs) else "—"
        v = volts[i] if isinstance(volts, list) and i < len(volts) else "—"
        def ff(val, p=2):
            return f"{val:.{p}f}" if isinstance(val, (int, float)) else str(val)
        lines.append(f"| J{i+1} | {ang:+.4f} | {ff(t)} | {ff(c)} | {ff(v)} |")
    lines.append("\n> 关节角度单位 **度 (°)**（`rm_current_arm_state_t.joint`）；温度/电流/电压来自 `rm_get_current_joint_temperature/current/voltage`。\n")

    lines.append("## 原始数据（JSON）\n")
    lines.append("```json")
    raw = {
        "arm_type": arm_type,
        "arm_dof": arm_dof,
        "state": state,
        "joint_temperature_C": temps if isinstance(temps, list) else None,
        "joint_current_mA": currs if isinstance(currs, list) else None,
        "joint_voltage_V": volts if isinstance(volts, list) else None,
        "software_version": str(swver) if swver else None,
    }
    lines.append(json.dumps(raw, ensure_ascii=False, indent=2))
    lines.append("```\n")

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(OUT_MD, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"\n[arm] 已写入 {OUT_MD}")

    try:
        arm.rm_delete_robot_arm()
    except Exception:
        pass


if __name__ == "__main__":
    main()
