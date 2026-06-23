#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
采集 Intel RealSense D435 相机的详细信息与图像，按 SN 号分别保存。

输出结构：
    cameras/<SN>/info.md        相机详细信息（markdown）
    cameras/<SN>/raw_params.json 相机原始参数（完整、未加工的 JSON）
    cameras/<SN>/rgb.png        RGB 彩色图
    cameras/<SN>/depth.png      深度图（彩色可视化）
    cameras/<SN>/depth_raw.png  深度图（原始 16-bit，保留真实米制数据）
"""

import os
import sys
import json
import math
import time
import datetime
import traceback

import numpy as np
import cv2
import pyrealsense2 as rs

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_ROOT = os.path.join(SCRIPT_DIR, "cameras")

# 采集分辨率 / 帧率
WIDTH, HEIGHT, FPS = 1280, 720, 30
# 预热帧数（等待自动曝光 / 白平衡收敛）
WARMUP_FRAMES = 30


def fov_deg(intr):
    """根据内参计算水平/垂直视场角(度)。"""
    hfov = 2 * math.degrees(math.atan(intr.width / (2.0 * intr.fx)))
    vfov = 2 * math.degrees(math.atan(intr.height / (2.0 * intr.fy)))
    return hfov, vfov


def safe_info(obj, key):
    """安全读取 device/sensor 的 camera_info 字段。"""
    try:
        return obj.get_info(key)
    except Exception:
        return None


def list_stream_profiles(sensor):
    """枚举传感器支持的流配置，按 (流类型, 格式, 宽, 高) 聚合可选帧率。"""
    profiles = {}
    for p in sensor.get_stream_profiles():
        v = p.as_video_stream_profile()
        st = str(p.stream_type()).replace("stream.", "")
        fmt = str(p.format()).replace("format.", "")
        key = (st, fmt, v.width(), v.height())
        profiles.setdefault(key, set()).add(p.fps())
    # 排序：流类型 -> 宽(降) -> 高(降)
    result = []
    for (st, fmt, w, h), fps_set in sorted(
        profiles.items(), key=lambda kv: (kv[0][0], -kv[0][2], -kv[0][3])
    ):
        result.append((st, fmt, w, h, sorted(fps_set)))
    return result


STRUCTURED_KEYS = {"sensors", "color_intrinsics", "depth_intrinsics",
                   "depth_scale", "depth_stats", "extrinsics_depth_to_color"}

# 基础字段的友好标签与展示顺序（其余字段追加在后面）
FIELD_LABELS = {
    "name": "设备名称",
    "serial_number": "序列号 (SN)",
    "product_line": "产品线",
    "product_id": "产品 ID",
    "firmware_version": "固件版本",
    "recommended_firmware_version": "推荐固件版本",
    "asic_serial_number": "ASIC 序列号",
    "advanced_mode": "Advanced Mode",
    "camera_locked": "相机锁定",
    "connection_type": "连接类型",
    "usb_type_descriptor": "USB 描述符",
    "physical_port": "物理端口",
    "firmware_update_id": "固件升级 ID",
    "ip_address": "IP 地址",
    "imu_type": "IMU 类型",
    "smcu_fw_version": "SMCU 固件版本",
    "mipi_driver_version": "MIPI 驱动版本",
    "debug_op_code": "调试操作码",
    "dfu_device_path": "DFU 设备路径",
}
FIELD_ORDER = list(FIELD_LABELS.keys())


def gather_device_info(device, color_intr=None, depth_intr=None,
                       depth_scale=None, depth_stats=None, extr=None):
    """收集设备所有可读取的信息，返回字典。动态遍历 camera_info 全部字段，
    兼容不同版本的 pyrealsense2。"""
    ci = rs.camera_info
    info = {}
    for attr in dir(ci):
        if attr.startswith("_") or attr == "value":
            continue
        val = safe_info(device, getattr(ci, attr))
        if val not in (None, ""):
            info[attr] = val
    # 传感器与支持的流
    sensors = []
    for s in device.query_sensors():
        sensors.append({
            "name": safe_info(s, ci.name),
            "profiles": list_stream_profiles(s),
        })
    info["sensors"] = sensors
    info["color_intrinsics"] = color_intr
    info["depth_intrinsics"] = depth_intr
    info["depth_scale"] = depth_scale
    info["depth_stats"] = depth_stats
    info["extrinsics_depth_to_color"] = extr
    return info


def capture(device_serial):
    """对单台相机：配置流、采集一帧 RGB+深度，返回帧数据与内参。"""
    cfg = rs.config()
    cfg.enable_device(device_serial)
    cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

    pipe = rs.pipeline()
    profile = pipe.start(cfg)
    try:
        # 预热，等待 AE/AWB 稳定
        for _ in range(WARMUP_FRAMES):
            pipe.wait_for_frames()
        frameset = pipe.wait_for_frames()

        depth_frame = frameset.get_depth_frame()
        color_frame = frameset.get_color_frame()
        if not depth_frame or not color_frame:
            raise RuntimeError("未能获取深度帧或彩色帧")

        depth_profile = profile.get_stream(rs.stream.depth)
        color_profile = profile.get_stream(rs.stream.color)
        depth_intr = depth_profile.as_video_stream_profile().get_intrinsics()
        color_intr = color_profile.as_video_stream_profile().get_intrinsics()
        extr = depth_profile.get_extrinsics_to(color_profile)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # 深度统计（剔除 0 值）
        valid = depth_image[depth_image > 0]
        if valid.size > 0:
            depth_stats = {
                "min_m": float(valid.min()) * depth_scale,
                "max_m": float(valid.max()) * depth_scale,
                "mean_m": float(valid.mean()) * depth_scale,
                "valid_ratio": float(valid.size) / depth_image.size,
            }
        else:
            depth_stats = None

        return {
            "color_image": color_image,
            "depth_image": depth_image,
            "color_intr": color_intr,
            "depth_intr": depth_intr,
            "extr": extr,
            "depth_scale": depth_scale,
            "depth_stats": depth_stats,
        }
    finally:
        pipe.stop()


def colorize_depth(depth_image):
    """把 16-bit 深度图转成彩色可视化（JET colormap，百分位归一化，0 值置黑）。"""
    valid = depth_image[depth_image > 0]
    if valid.size == 0:
        return np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)
    lo, hi = np.percentile(valid, [1, 99])
    if hi - lo < 1e-6:
        hi = lo + 1e-6
    d8 = np.clip((depth_image.astype(np.float32) - lo) * 255.0 / (hi - lo), 0, 255).astype(np.uint8)
    depth_color = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
    depth_color[depth_image == 0] = 0
    return depth_color


def render_md(info, paths, ts):
    sn = info["serial_number"]
    lines = []
    add = lines.append

    add(f"# 相机信息 — {info.get('name') or ''}")
    add("")
    add(f"> 采集时间：{ts}  ")
    add("")
    add("## 基本信息")
    add("")
    add("| 项目 | 值 |")
    add("| --- | --- |")
    # 按预定义顺序渲染基础字段，其余动态字段追加在后面
    rendered = set()
    for key in FIELD_ORDER:
        if key in info:
            label = FIELD_LABELS[key]
            value = info[key]
            # SN 加粗、端口类字段用反引号包裹便于阅读
            if key == "serial_number":
                value = f"**{value}**"
            elif key in ("physical_port", "firmware_update_id", "usb_type_descriptor",
                         "connection_type", "debug_op_code", "dfu_device_path"):
                value = f"`{value}`"
            add(f"| {label} | {value} |")
            rendered.add(key)
    for key, value in info.items():
        if key in STRUCTURED_KEYS or key in rendered:
            continue
        add(f"| {key} | `{value}` |")
    add("")

    # 传感器与支持的流
    add("## 传感器与支持的流配置")
    add("")
    for s in info["sensors"]:
        add(f"### {s['name']}")
        add("")
        add("| 流类型 | 格式 | 宽×高 | 可选帧率 (fps) |")
        add("| --- | --- | --- | --- |")
        for st, fmt, w, h, fps in s["profiles"]:
            add(f"| {st} | {fmt} | {w}×{h} | {', '.join(str(f) for f in fps)} |")
        add("")

    # 内参
    ci, di = info["color_intrinsics"], info["depth_intrinsics"]
    add("## 实时标定参数（本次采集分辨率）")
    add("")
    add("### 彩色 (Color) 内参")
    add("")
    add(f"- 分辨率：{ci.width}×{ci.height}")
    add(f"- 焦距：fx={ci.fx:.4f}, fy={ci.fy:.4f}")
    add(f"- 主点：ppx={ci.ppx:.4f}, ppy={ci.ppy:.4f}")
    add(f"- 畸变模型：{ci.model}")
    add(f"- 畸变系数：{[round(c, 6) for c in ci.coeffs]}")
    hfov, vfov = fov_deg(ci)
    add(f"- 视场角：HFOV≈{hfov:.2f}°，VFOV≈{vfov:.2f}°")
    add("")

    add("### 深度 (Depth) 内参")
    add("")
    add(f"- 分辨率：{di.width}×{di.height}")
    add(f"- 焦距：fx={di.fx:.4f}, fy={di.fy:.4f}")
    add(f"- 主点：ppx={di.ppx:.4f}, ppy={di.ppy:.4f}")
    add(f"- 畸变模型：{di.model}")
    add(f"- 畸变系数：{[round(c, 6) for c in di.coeffs]}")
    hfov, vfov = fov_deg(di)
    add(f"- 视场角：HFOV≈{hfov:.2f}°，VFOV≈{vfov:.2f}°")
    add("")

    ds = info["depth_scale"]
    add(f"- 深度单位 (depth_scale)：{ds:.6e} 米/刻度（即 {1.0/ds:.0f} 刻度/米）")
    add("")

    # 深度统计
    st = info["depth_stats"]
    if st:
        add("### 本次深度帧统计（有效像素）")
        add("")
        add(f"- 最近距离：{st['min_m']:.3f} m")
        add(f"- 最远距离：{st['max_m']:.3f} m")
        add(f"- 平均距离：{st['mean_m']:.3f} m")
        add(f"- 有效像素占比：{st['valid_ratio']*100:.1f}%")
        add("")

    # 外参
    ex = info["extrinsics_depth_to_color"]
    add("### 深度→彩色 外参")
    add("")
    add("- 旋转矩阵 R（行优先）：")
    add("```")
    for r in range(3):
        row = ex.rotation[r * 3:(r + 1) * 3]
        add("  " + "  ".join(f"{x:+.6f}" for x in row))
    add("```")
    add(f"- 平移向量 T：[{ex.translation[0]:+.6f}, {ex.translation[1]:+.6f}, {ex.translation[2]:+.6f}] (米)")
    add("")

    # 采集的图像
    add("## 采集的图像")
    add("")
    add("### RGB 彩色图")
    add("")
    add(f"![RGB]({os.path.basename(paths[0])})")
    add("")
    add("### 深度图（彩色可视化）")
    add("")
    add(f"![Depth]({os.path.basename(paths[1])})")
    add("")
    add("### 深度图（原始 16-bit，数值=米/depth_scale）")
    add("")
    add(f"![Depth Raw]({os.path.basename(paths[2])})")
    add("")

    return "\n".join(lines)


def _intrinsics_to_dict(intr):
    """把 rs.intrinsics 序列化为原始字典。"""
    if intr is None:
        return None
    return {
        "width": int(intr.width),
        "height": int(intr.height),
        "ppx": float(intr.ppx),
        "ppy": float(intr.ppy),
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "model": str(intr.model),
        "coeffs": [float(c) for c in intr.coeffs],
    }


def _extrinsics_to_dict(extr):
    """把 rs.extrinsics 序列化为原始字典。"""
    if extr is None:
        return None
    return {
        "rotation": [float(x) for x in extr.rotation],
        "translation": [float(x) for x in extr.translation],
    }


def _sensor_options(sensor):
    """枚举传感器全部 option：当前值、量程(min/max/step/default)、只读标记、描述。"""
    options = []
    for opt in sensor.get_supported_options():
        entry = {"name": str(opt).replace("option.", "")}
        try:
            entry["description"] = sensor.get_option_description(opt)
        except Exception:
            entry["description"] = None
        try:
            entry["value"] = float(sensor.get_option(opt))
        except Exception:
            entry["value"] = None
        try:
            rng = sensor.get_option_range(opt)
            entry["min"] = float(rng.min)
            entry["max"] = float(rng.max)
            entry["step"] = float(rng.step)
            entry["default"] = float(rng.default)
        except Exception:
            entry.update({"min": None, "max": None, "step": None, "default": None})
        try:
            entry["readonly"] = bool(sensor.is_option_read_only(opt))
        except Exception:
            entry["readonly"] = None
        options.append(entry)
    return options


def _sensor_stream_profiles(sensor):
    """枚举传感器全部流配置（逐条、未聚合），保留原始粒度。"""
    out = []
    for p in sensor.get_stream_profiles():
        v = p.as_video_stream_profile()
        out.append({
            "stream": str(p.stream_type()).replace("stream.", ""),
            "format": str(p.format()).replace("format.", ""),
            "index": p.stream_index(),
            "width": v.width(),
            "height": v.height(),
            "fps": p.fps(),
        })
    return out


def _struct_to_dict(s):
    """把 advanced-mode 的参数组结构体序列化为字典（数值字段转 float）。"""
    out = {}
    for attr in dir(s):
        if attr.startswith("_"):
            continue
        try:
            val = getattr(s, attr)
        except Exception:
            continue
        if callable(val):
            continue
        if isinstance(val, (int, float)):
            val = float(val)
        out[attr] = val
    return out


def _advanced_mode_groups(adv):
    """遍历 advanced-mode 全部 get_* 分组，序列化为 {组名: 参数字典}。"""
    groups = {}
    for m in dir(adv):
        if not m.startswith("get_") or m.startswith("get_json"):
            continue
        fn = getattr(adv, m)
        if not callable(fn):
            continue
        name = m[len("get_"):]
        try:
            struct = fn()
            groups[name] = _struct_to_dict(struct)
        except Exception as e:
            groups[name] = f"<error: {e}>"
    return groups


def dump_raw_params(device, cap, ts):
    """采集设备全部原始参数，返回可 JSON 序列化的字典。

    包含：device_info(camera_info 全字段)、Advanced Mode 开关与原始 JSON 参数树、
    每个 sensor 的 options(当前值/量程/默认/只读) 与流配置、原始内参/外参/depth_scale。
    """
    ci = rs.camera_info

    # 1) camera_info 全字段（原始字符串）
    device_info = {}
    for attr in dir(ci):
        if attr.startswith("_") or attr == "value":
            continue
        val = safe_info(device, getattr(ci, attr))
        if val is not None and val != "":
            device_info[attr] = val

    # 2) Advanced Mode：开关 + 各参数分组
    adv_enabled = None
    adv_groups = None
    try:
        adv = rs.rs400_advanced_mode(device)
        adv_enabled = bool(adv.is_enabled())
        adv_groups = _advanced_mode_groups(adv)
    except Exception as e:
        adv_enabled = f"<error: {e}>"

    # 3) 每个 sensor 的 options 与流配置
    sensors = []
    for s in device.query_sensors():
        sensors.append({
            "name": safe_info(s, ci.name),
            "options": _sensor_options(s),
            "stream_profiles": _sensor_stream_profiles(s),
        })

    raw = {
        "capture_time": ts,
        "device_info": device_info,
        "advanced_mode_enabled": adv_enabled,
        "advanced_mode_params": adv_groups,
        "sensors": sensors,
        "calibration": {
            "depth_scale": float(cap["depth_scale"]),
            "color_intrinsics": _intrinsics_to_dict(cap["color_intr"]),
            "depth_intrinsics": _intrinsics_to_dict(cap["depth_intr"]),
            "depth_to_color_extrinsics": _extrinsics_to_dict(cap["extr"]),
        },
        "depth_frame_stats": cap["depth_stats"],
    }
    return raw


def main():
    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    os.makedirs(OUT_ROOT, exist_ok=True)

    ctx = rs.context()
    devices = list(ctx.query_devices())
    if not devices:
        print("未检测到任何 RealSense 设备。", file=sys.stderr)
        sys.exit(1)

    print(f"检测到 {len(devices)} 台设备，开始逐台采集...")
    failures = []
    for idx, dev in enumerate(devices):
        sn = safe_info(dev, rs.camera_info.serial_number) or f"unknown_{idx}"
        print(f"\n[{idx}] 处理 SN={sn} ...")
        try:
            cap = capture(sn)
            out_dir = os.path.join(OUT_ROOT, sn)
            os.makedirs(out_dir, exist_ok=True)

            # 保存图像：RGB、深度彩色可视化、深度原始 16-bit
            color_image = cap["color_image"]
            depth_image = cap["depth_image"]

            rgb_path = os.path.join(out_dir, "rgb.png")
            depth_vis_path = os.path.join(out_dir, "depth.png")
            depth_raw_path = os.path.join(out_dir, "depth_raw.png")

            cv2.imwrite(rgb_path, color_image)
            cv2.imwrite(depth_vis_path, colorize_depth(depth_image))
            cv2.imwrite(depth_raw_path, depth_image)

            info = gather_device_info(
                dev,
                color_intr=cap["color_intr"],
                depth_intr=cap["depth_intr"],
                depth_scale=cap["depth_scale"],
                depth_stats=cap["depth_stats"],
                extr=cap["extr"],
            )
            md = render_md(info, (rgb_path, depth_vis_path, depth_raw_path), ts)
            md_path = os.path.join(out_dir, "info.md")
            with open(md_path, "w", encoding="utf-8") as f:
                f.write(md)

            # 原始参数 dump（完整、未加工）
            raw = dump_raw_params(dev, cap, ts)
            raw_path = os.path.join(out_dir, "raw_params.json")
            with open(raw_path, "w", encoding="utf-8") as f:
                json.dump(raw, f, ensure_ascii=False, indent=2)

            print(f"    完成 → {out_dir}/  (info.md, raw_params.json, rgb.png, depth.png, depth_raw.png)")
        except Exception as e:
            print(f"    采集失败：{e}", file=sys.stderr)
            traceback.print_exc()
            failures.append(sn)

    print("\n===== 汇总 =====")
    print(f"成功输出目录：{OUT_ROOT}")
    if failures:
        print(f"失败的相机 SN：{failures}")
        sys.exit(1)


if __name__ == "__main__":
    main()
