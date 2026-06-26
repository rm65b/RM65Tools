# RealMan SDK 关节状态读取方法参考

`Robotic_Arm` (robotic-arm==1.1.5，conda env `rm65`) 读取机械臂状态的方法中，**部分方法名具有误导性**，容易写错。本文档记录实测的返回结构与坑点，避免重复踩雷。

> 测试环境：RM65-B（6 自由度），连接 `192.168.1.18:8080`，`arm_dof=6`。

---

## ⚠️ 误导性方法名（重点）

| 方法名 | 望文生义（错） | 实际含义 |
|---|---|---|
| `rm_get_current_joint_current()` | "当前关节角" | **关节电流 (mA)** ❌ |

这里的 `current` 指 **electrical current（电流）**，不是 "当前的"。**不要用它拿关节角度。**

---

## 推荐用法

### ✅ 一次拿全关节角 + 末端位姿

```python
ret, state = arm.rm_get_current_arm_state()
# ret: 0=成功
# state: dict
#   'joint': [6 关节角度, 单位:度]
#   'pose' : [x, y, z, rx, ry, rz]   末端位姿
#   'err'  : {'err_len': n, 'err': [...]}
```

**首选**。一次网络往返同时拿到关节角与位姿，比 "读角度 + 正运动学算位姿" 更准更快。

### ✅ 仅需关节角度（度）

```python
ret, joint_deg = arm.rm_get_joint_degree()
# ret: 0=成功
# joint_deg: [6 关节角度, 单位:度]
```

---

## ❌ 正运动学方法的返回类型陷阱

```python
# 错误写法 —— 会抛异常
ret, pose = arm.rm_algo_forward_kinematics(joint)   # ValueError: too many values to unpack (expected 2)
```

`rm_algo_forward_kinematics(joint, flag=1)` 的返回类型是 **裸 `list[float]`（位姿）**，**不是 `(code, pose)` 元组**。直接按元组解包会报 `too many values to unpack (expected 2)`。

```python
# 正确写法
pose = arm.rm_algo_forward_kinematics(joint)   # pose = [x, y, z, rx, ry, rz]
```

> 通常无需手算正运动学——`rm_get_current_arm_state()` 返回的 `pose` 字段就是控制器给出的末端位姿。

---

## 实测读数示例

```python
ret, state = arm.rm_get_current_arm_state()
# joint = [-69.72, 0.69, 85.18, 5.46, 71.26, -100.5]   # 度
# pose  = [-0.105, 0.247, 0.380, -2.732, 0.022, 0.529]  # [x,y,z,rx,ry,rz]
```

---

## 版本说明

- SDK 版本：**1.1.5**（`current c api version: v1.1.5`）。
- **不存在** `rm_get_current_joint_angle` / `rm_get_joint_angle`（其它版本可能有）。
  [`get_arm_state.py`](../src/examples/get_arm_state.py) 的 `safe_call()` 用 `getattr` + 防御式解包来兼容不同版本，可作为参考写法。
- 状态码统一约定：`0`=成功；`-1`/`-2`/`-3`=通信/解析失败；部分方法 `1`=控制器返回 false。

---

## 相关

- 轨迹记录/回放工具：[`src/examples/test_trajectory.py`](../src/examples/test_trajectory.py)
- 只读状态检查工具：[`src/examples/get_arm_state.py`](../src/examples/get_arm_state.py)
- 抓取实现参考：[`PICK_PLACE_REFERENCE.md`](PICK_PLACE_REFERENCE.md)
