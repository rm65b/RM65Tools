# RM65 项目完整指南

本文档整合了 RM65-B 机械臂控制项目的所有组件和使用说明。

## 项目概览

```
RM65_robot/
├── prj1.raspi.ctrl/
│   ├── src/
│   │   ├── core/                    # 核心模块
│   │   │   ├── camera_controller.py # RealSense D435 相机控制
│   │   │   └── vision.py            # 高级物体检测 (ArUco, 颜色, 深度)
│   │   ├── examples/                # 示例程序
│   │   │   ├── test_arm.py              # 机械臂基础测试
│   │   │   ├── test_camera.py           # 相机功能测试
│   │   │   ├── test_pick_place.py       # ⭐ 抓取放置演示
│   │   │   ├── test_hand_eye_calibration.py  # 手眼标定工具
│   │   │   └── test_trajectory.py        # 轨迹记录/回放
│   │   ├── control/                 # 控制模块 (待扩展)
│   │   └── drivers/                 # 驱动模块 (待扩展)
│   └── docs/                       # 文档
│       ├── PICK_PLACE_REFERENCE.md # 抓取实现参考
│       └── SDK_STATE_METHODS.md    # SDK 关节状态读取方法(名称有坑)
└── README.md
```

## 快速开始

### 1. 环境配置

```bash
# 创建 conda 环境
conda create -n rm65 python=3.10
conda activate rm65

# 安装依赖
pip install pyrealsense2 opencv-python numpy scipy
```

### 2. 测试连接

```bash
# 测试机械臂连接
python prj1.raspi.ctrl/src/examples/test_arm.py

# 测试相机
python prj1.raspi.ctrl/src/examples/test_camera.py
```

### 3. 运行演示

```bash
# 抓取放置演示
python prj1.raspi.ctrl/src/examples/test_pick_place.py

# 手眼标定
python prj1.raspi.ctrl/src/examples/test_hand_eye_calibration.py

# 轨迹回放
python prj1.raspi.ctrl/src/examples/test_trajectory.py replay
```

## 模块说明

### camera_controller.py

RealSense D435 相机控制器,提供:

- RGB-D 图像获取
- 像素坐标转 3D 坐标
- 点云生成

```python
from core.camera_controller import RealSenseD435

with RealSenseD435() as camera:
    color, depth = camera.get_frame()
    x, y, z = camera.deproject_pixel(pixel_x, pixel_y, depth_val)
```

### vision.py

高级物体检测模块,支持:

- **ArUco 标记检测**: 用于精确定位
- **颜色检测**: HSV 颜色空间分割
- **边缘检测**: Canny 边缘检测
- **深度分割**: 基于深度图的区域分割

```python
from core.vision import AdvancedDetector, DetectionMethod

detector = AdvancedDetector()
detector.add_color_detector("red", (0, 100, 100), (10, 255, 255))

results = detector.detect_all(color_image, depth_image)
aruco_detections = results[DetectionMethod.ARUCO]
```

### test_pick_place.py

完整的抓取放置演示,包含:

- **RM65Controller**: 机械臂控制封装
  - 关节运动 (`movej`)
  - 直线运动 (`movel`)
  - 夹爪控制

- **VisionSystem**: 视觉系统
  - 物体检测
  - 深度获取

- **PickPlaceDemo**: 抓取放置流程

### test_hand_eye_calibration.py

手眼标定工具,计算相机到机器人的变换矩阵。

**标定流程:**

1. 准备固定在工作空间的特征点
2. 移动机器人使末端对准特征点
3. 在相机图像中点击该特征点
4. 重复 2-3 步,至少 4 个不同角度
5. 计算变换矩阵并保存

**使用方法:**

```bash
python test_hand_eye_calibration.py
# 按提示操作,完成至少4个标定点
```

**标定文件保存位置:**
```
tests/output/hand_eye_calibration.json
```

### test_trajectory.py

轨迹记录与回放工具。

**记录轨迹:**

```bash
python test_trajectory.py record --duration 30 --rate 10
# 或手动记录 (Ctrl+C 停止)
python test_trajectory.py record
```

**回放轨迹:**

```bash
# 列出可用轨迹
python test_trajectory.py list

# 回放指定轨迹
python test_trajectory.py replay --file <path> --speed 1.0
```

## 工作流程建议

### 完整的视觉引导抓取流程

```
┌─────────────────┐
│ 1. 手眼标定     │
│ 计算相机-机器人 │
│ 变换矩阵        │
└────────┬────────┘
         │
┌────────▼────────┐
│ 2. 记录/示教     │
│ 记录抓取/放置    │
│ 轨迹             │
└────────┬────────┘
         │
┌────────▼────────┐
│ 3. 视觉检测     │
│ 检测目标物体     │
│ 获取3D位置       │
└────────┬────────┘
         │
┌────────▼────────┐
│ 4. 坐标变换     │
│ 相机坐标 →      │
│ 机器人坐标       │
└────────┬────────┘
         │
┌────────▼────────┐
│ 5. 执行抓取     │
│ 移动 → 夹爪 →   │
│ 移动 → 释放      │
└─────────────────┘
```

### 典型使用场景

#### 场景 1: 基于 ArUco 的精确抓取

```python
from core.vision import ArUcoDetector
from examples.test_pick_place import RM65Controller

# 1. 在物体上粘贴 ArUco 标记
# 2. 检测标记
aruco = ArUcoDetector(dict_type='6x6')
detections = aruco.detect(color_image)

# 3. 获取深度并计算位置
for det in detections:
    depth = camera.get_depth_at_pixel(depth, det.center_x, det.center_y)
    x, y, z = camera.deproject_pixel(det.center_x, det.center_y, depth)
    # 使用手眼标定结果转换到机器人坐标
```

#### 场景 2: 轨迹示教与回放

```bash
# 1. 手动示教抓取动作
python test_trajectory.py record --duration 10 --name pick_motion

# 2. 手动示教放置动作
python test_trajectory.py record --duration 10 --name place_motion

# 3. 回放组合动作
python test_trajectory.py replay --file pick_motion.json
python test_trajectory.py replay --file place_motion.json
```

#### 场景 3: 颜色分类抓取

```python
from core.vision import AdvancedDetector

# 设置不同颜色的检测器
detector = AdvancedDetector()
detector.add_color_detector("red", (0, 100, 100), (10, 255, 255))
detector.add_color_detector("blue", (100, 100, 100), (130, 255, 255))

# 检测并分类
results = detector.detect_all(color_image, depth_image)
red_objects = results['color_red']
blue_objects = results['color_blue']
```

## 安全注意事项

⚠️ **操作前务必确认:**

1. 工作空间无人员
2. 机械臂已正确安装和固定
3. 有紧急停止准备
4. 首次运行时使用低速测试

## API 参考

### 机械臂控制 (RoboticArm)

```python
# 连接
arm = RoboticArm(mode=rm_thread_mode_e.RM_SINGLE_MODE_E)
handle = arm.rm_create_robot_arm("192.168.1.18", 8080, level=2)

# 运动控制
arm.rm_movej(joint_angles, speed, accel, type)  # 关节运动
arm.rm_movel(pose, speed, accel, type)        # 直线运动

# 夹爪
arm.rm_set_gripper_pick_on()    # 闭合
arm.rm_set_gripper_release()    # 松开

# 状态获取(推荐一次拿全关节角+位姿)
ret, state = arm.rm_get_current_arm_state()   # state: {'joint':[...], 'pose':[...], 'err':...}
ret, joint = arm.rm_get_joint_degree()        # 关节角度(度)

# ⚠️ 名称误导: rm_get_current_joint_current() 返回关节电流(mA), 不是角度!
# ⚠️ rm_algo_forward_kinematics(joints) 返回裸 list[float], 不是 (code, pose) 元组
# 详见 docs/SDK_STATE_METHODS.md
```

### 相机控制 (RealSenseD435)

```python
camera = RealSenseD435(width=640, height=480, fps=30)
camera.connect()

color, depth = camera.get_frame()
depth_val = camera.get_depth_at_pixel(depth, x, y)
x_3d, y_3d, z_3d = camera.deproject_pixel(x, y, depth_val)
```

## 故障排除

### 问题: 机械臂连接失败

**可能原因:**
- IP 地址错误
- 网络不通
- 机械臂未上电

**解决方法:**
```bash
# 检查网络
ping 192.168.1.18

# 检查机械臂状态 (使用官方工具)
```

### 问题: 相机连接失败

**可能原因:**
- USB 权限问题
- 驱动未安装

**解决方法:**
```bash
# 添加 USB 规则
sudo nano /etc/udev/rules.d/realsense.rules
# 添加: SUBSYSTEM=="usb", ATTR{idVendor}=="8086", MODE="0666"

# 重载规则
sudo udevadm control --reload-rules
```

### 问题: 检测不准确

**解决方法:**
1. 调整光照条件
2. 调整检测参数 (HSV 范围、深度范围)
3. 使用 ArUco 标记获得更高精度

## 参考资源

- [RealMan Robot GitHub](https://github.com/RealManRobot/rm_robot)
- [RealSense Python SDK](https://github.com/IntelRealSense/librealsense)
- [ArUco 标记生成](https://chev.me/arucogen/)
- [OpenCV 文档](https://docs.opencv.org/)
