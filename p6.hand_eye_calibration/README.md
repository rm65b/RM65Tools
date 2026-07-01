# RM65B 手眼标定

针对 **睿尔曼 RM65B 机械臂 + Intel RealSense D435 相机** 的手眼标定工具。支持两种安装方式:

- `eye_in_hand`(眼在手上):相机装在机械臂末端,标定 **相机 → 末端(cam2end)**。
- `eye_to_hand`(眼在手外):相机固定在外部,标定 **相机 → 基座(cam2base)**。

两种模式最终都归结为求解 `AX = XB`。

## 工作流程

标定分三步:**采集数据 → 计算参数 → 验证精度**,分别对应三个脚本。

```
collect_data.py ── 采集图片 + 机械臂位姿 ─▶ eye_hand_data/dataYYYYMMDD[NN]/
compute.py      ── 解 AX=XB               ─▶ hand_eye_result_{mode}.npz
validate.py     ── 控制机械臂触碰目标点    ─▶ 打印偏差
```

## 依赖

```sh
pip install -r requirements.txt
```

依赖:`numpy`、`opencv-python`、`pyrealsense2`、`scipy`、`PyYAML`。

## 配置

修改 [config.yaml](./config.yaml):

```yaml
checkerboard_args:
  XX : 9      # 标定板长度方向内角点数
  YY : 7      # 标定板宽度方向内角点数
  L  : 0.03   # 单格边长(米)
calib_mode : eye_to_hand   # eye_in_hand / eye_to_hand
```

> `XX/YY` 必须与实际标定板的**内角点数**一致,否则角点检测失败。

`calib_mode` 是**整个流程的工作模式开关**:设一次,采集 / 计算 / 验证都默认跟随它。三个脚本均以 config.yaml 为默认,`--mode` 仅作临时覆盖。

机械臂默认连接地址:依次尝试 `192.168.1.18` / `192.168.10.18`,端口 `8080`。

## 1. 采集数据

```sh
python collect_data.py
```

- 打开 D435(640×480)实时画面,按键 **`s`** 采集一帧。
- 每帧**同时**保存:`N.jpg`(图片)和 `poses.txt` 中追加一行位姿 `x,y,z,rx,ry,rz`。
- 位姿单位自动换算:位置 `÷1e6`(0.001mm→米),姿态 `÷1e3`(0.001rad→弧度)。
- 若获取位姿失败(拖动示教超速 `4105`、碰撞 `4109` 等),会自动 `clear_arm_err` 清错后重试一次;**仅在成功保存后编号才自增**,保证图片与位姿严格一一对应。
- 采集时把 `config.yaml` 中的 `calib_mode` 写入数据夹的 `mode.txt`,供 `compute.py` 自动识别。

采集建议:15~20 组以上、姿态尽量多样(覆盖不同旋转),标定板需完整出现在画面中。

## 2. 计算标定参数

```sh
python compute.py                                          # 自动:最新数据夹 + mode.txt/config 模式
python compute.py --data data2026062904 --mode eye_to_hand # 指定数据夹与模式
```

**模式 / 数据夹确定优先级**(从高到低):
`--mode` / `--data` 命令行参数 > 数据夹内 `mode.txt` > `config.yaml`。

流程:棋盘格角点提取(亚像素)→ `cv2.calibrateCamera` 得标定板在相机系位姿 → `cv2.calibrateHandEye`(TSAI)解 `AX=XB`。`eye_to_hand` 时机械臂位姿取逆。

结果按模式分别保存,**互不覆盖**:
- `hand_eye_result_eye_in_hand.npz`
- `hand_eye_result_eye_to_hand.npz`

每个文件含 `R`(3×3 旋转矩阵)、`t`(3 平移向量,米)、`quaternion`(x,y,z,w)、`mode`。终端会打印旋转矩阵、平移向量、四元数和相机标定重投影误差。

## 3. 验证精度

```sh
# 默认读 config.yaml 的 calib_mode 选择对应结果
python validate.py --cam -0.067 -0.048 0.783 --gripper 0.2515 --above 0.01 --clearance 0.005

# 临时覆盖模式(不影响 config.yaml)
python validate.py --mode eye_to_hand --cam -0.067 -0.048 0.783 --gripper 0.2515

# 只算不动(预览目标位姿,不发运动指令)
python validate.py --cam -0.067 -0.048 0.783 --gripper 0.2515 --dry-run

# 指定接近姿态(如夹爪朝下)
python validate.py --cam -0.067 -0.048 0.783 --gripper 0.2515 --rpy 3.1416 0 0
```

| 参数 | 说明 | 默认 |
| --- | --- | --- |
| `--cam X Y Z` | 相机坐标系下的目标点(米);不传则交互输入 | — |
| `--mode` | 临时覆盖标定模式;默认读 config.yaml 的 `calib_mode`,其次自动选唯一存在的结果 | 读 config.yaml |
| `--gripper` | 夹爪长度(米) | `0.20` |
| `--clearance` | 尖端停在物体上方的间隙(米) | `0.005` |
| `--above` | 先到的正上方高度(米) | `0.10` |
| `--rpy RX RY RZ` | 接近姿态(弧度);不传则沿用当前姿态 | 当前姿态 |
| `--speed` | 运动速度百分比 | `20` |
| `--ip` / `--port` | 机械臂地址 | `192.168.1.18` / `8080` |
| `--dry-run` | 仅计算打印目标,不运动 | — |

原理:把相机系点 `p_cam` 转到基座系,机械臂先到正上方再垂直下放,使夹爪尖端到达目标点,读回实际尖端位置并报告 XY / 三轴偏差。

## 目录结构

```
p6.hand_eye_calibration/
├── config.yaml              # 棋盘格参数 + 标定模式
├── collect_data.py          # 采集图片与位姿
├── compute.py               # 计算手眼标定
├── validate.py              # 验证标定精度
├── hand_eye_result_*.npz    # 标定结果(按模式分文件)
├── RobotToolPose.csv        # compute.py 生成的位姿齐次矩阵(中间产物)
├── eye_hand_data/           # 采集数据:每夹含 1.jpg..N.jpg + poses.txt + mode.txt
└── libs/                    # auxiliary(网络/文件夹) + poses(位姿↔矩阵) + log_setting(日志)
```

数据约定:
- `poses.txt`:每行 `x,y,z,rx,ry,rz`(米 / 弧度),与 `1.jpg..N.jpg` **顺序一一对应**。
- `mode.txt`:该批数据的标定模式。

## 参考

- [睿尔曼机械臂标定说明](./rm65b标定说明.md)
- [采集说明与排错](./采集说明与排错.md)
