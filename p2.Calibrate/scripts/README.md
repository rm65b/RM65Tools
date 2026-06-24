# RM65-B 双 RealSense D435 相机标定 · 脚本

配套 [标定方案.md](../标定方案.md) 的完整流水线：内参 → 采集 → 解算 → 验证。
两路最终统一到机械臂 **基座系**。

```
cameras/                         # 标定结果存档
├── 261722075459/  (眼在手上)       intrinsics.json  handeye.json  samples/
└── 261722076078/  (眼在手外)       intrinsics.json  handeye.json  samples/
scripts/
├── config.py                    # ★ 集中配置：先改这里
├── transforms.py  charuco.py  realsense_cam.py  arm_rmm.py
├── 01_collect.py                # 采集
├── 02_calibrate_intrinsics.py   # 内参复校（可选）
├── 03_solve_handeye.py          # 解算
└── 04_verify.py                 # 验证
```

## 0. 依赖与运行环境

推荐在独立 conda 环境（如 `rm65`）中运行，已实测通过：

```bash
conda create -n rm65 python=3.12 -y && conda activate rm65
# 视觉/数值（opencv-python 4.13 自带 aruco；scipy 不需要）
pip install numpy "opencv-python>=4.7" pyrealsense2
# 机械臂 SDK（采集需要；解算/验证离线部分不需要）
pip install Robotic_Arm           # RealMan 官方 Python SDK（Robotic_Arm.rm_robot_interface，实测 v1.1.5）
```

运行（二选一）：
```bash
conda run -n rm65 python scripts/01_collect.py --mode eye_in_hand   # 不激活
# 或先 conda activate rm65 再直接 python scripts/...
```

两台 D435 已识别：`261722075459`(眼在手上)、`261722076078`(眼在手外)。

## 1. 先改 `config.py`

- `ARM_IP` / `ARM_PORT`：机械臂 IP（默认 192.168.1.18:8080）
- 标定板：`BOARD_SQUARES_X/Y`、`BOARD_SQUARE_LEN_M`、`BOARD_MARKER_LEN_M`、`ARUCO_DICT`
  —— **板尺寸务必实测**（游标卡尺，误差<0.1mm），错了重投影/外参全错。
- `POSE_RPY_UNIT` / `POSE_RPY_ORDER`：位姿约定（见下“位姿约定”）。

## 2. 标定流程

### Step 0　制作标定板（先做一次）
```bash
conda run -n rm65 python scripts/00_make_board.py
```
输出 [board/board.pdf](../board/board.pdf)：板按**精确毫米**铺版（默认 7×5、方格 30mm / 标记 22mm、`DICT_5X5_100`）。
打印对话框选“实际尺寸 / 100%”（勿缩放），印后用卡尺复核每格=30mm，裱在硬板/铝板上。
自定义（同时改 `config.py`）：`--square-mm 25 --marker-mm 18 --sx 8 --sy 6`。

### Step 1　采集
```bash
# 眼在手上：标定板固定桌面，相机A随臂动
python scripts/01_collect.py --mode eye_in_hand
# 眼在手外：标定板固定在末端夹爪(随臂动)，相机B固定俯视
python scripts/01_collect.py --mode eye_to_hand
```
- **半自动**：用示教器/RM App 摆出多样位姿（绕 ≥2 个不平行轴旋转、`|q3|`/`|q5|>20°`、规避奇异），停稳后回车即采。
- 首次先自检：`python scripts/01_collect.py --mode eye_in_hand --inspect`
  打印当前位姿/内参/奇异校验。**若位姿解析异常，按输出调整 `arm_rmm._parse_state`。**
- 未装 SDK 时加 `--arm manual`，按提示粘贴法兰位姿即可跑通。

### Step 2（可选）内参复校
```bash
python scripts/02_calibrate_intrinsics.py --serial 261722075459
python scripts/02_calibrate_intrinsics.py --serial 261722076078
```
出厂内参对手眼已足够；仅当 `01` 的重投影偏大时复校。结果覆盖 `intrinsics.json`(source=recalibrated)。

### Step 3　解算
```bash
python scripts/03_solve_handeye.py --serial 261722075459   # → handeye.json (T_flange_cam)
python scripts/03_solve_handeye.py --serial 261722076078   # → handeye.json (T_base_cam)
```
四种方法各解，按“链一致性残差”挑最优；打印旋转/平移残差与重投影，并对照验收阈值。

### Step 4　验证
```bash
python scripts/04_verify.py --serial 261722075459          # 离线复核单路
python scripts/04_verify.py --live                         # 两路同框实时比对（需两路已解算+机械臂）
```

## 3. 位姿约定（最易错，务必确认）

`config.py` 默认（已与 SDK 实测对齐）：
- `POSE_RPY_UNIT = 'rad'`：RealMan SDK `rm_get_current_arm_state` 返回的 `pose=[x,y,z,rx,ry,rz]`
  欧拉角为 **弧度**（SDK 源码明确“单位：rad”）。
- `POSE_RPY_ORDER = 'xyz'`（外旋）：`R = Rz(rz) @ Ry(ry) @ Rx(rx)`，RPY=(rx,ry,rz)。
  关节角为度，用于奇异/限位检查。

判断对错：跑完 `03` 若**旋转一致性残差很大或 eye-to-hand 平移离谱**，多半是单位/顺序错了——
切换这两项重解；或用 `04_verify.py --live` 的物理触点/同框比对验证符号。

## 4. 结果用法（运行时）

读 `handeye.json` 的 `transform`（4×4）：
- 眼在手上：`P_base = T_base_flange(now) @ T_flange_cam @ P_camA`
- 眼在手外：`P_base = T_base_cam @ P_camB`

`T_base_flange(now)` 来自 RM65-B 实时正运动学（TCP 原点须在 d6=144mm 法兰面中心）。
两路相机由此共享同一基座系。

## 5. 常见问题（详见 标定方案.md §8）

| 现象 | 处理 |
|---|---|
| `01` 检测不到角点 | 板尺寸/字典配置错；或光照/反光/距离。 |
| 位姿解析/连接报错 | `--inspect` 看状态码与 `state_dict['pose']`；连接失败查 `config.ARM_IP/PORT` 与网段。 |
| 三方法残差差异大 | 位姿退化（缺轴向旋转/触奇异），补采。 |
| eye-to-hand 平移离谱 | 位姿单位(rad/deg)或 RPY 顺序错；或未取逆（本脚本已处理）。 |
| 两路同框差值大且方向固定 | 坐标系/符号约定错，优先查 `config.py` 位姿项。 |
