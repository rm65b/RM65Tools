# RM65 Pick-And-Place 实现说明

本文档对比参考实现 (MoveIt/ROS) 与 RealMan API 实现,并提供手眼标定指南。

## 参考实现分析

### 源文件
[api_rm65_pick_place_demo.cpp](https://github.com/RealManRobot/rm_robot/blob/main/rm_demo/src/api_rm65_pick_place_demo.cpp)

### 技术栈
- **ROS/MoveIt**: 使用 MoveIt 运动规划框架
- **Collision Objects**: 用于环境建模和碰撞检测
- **AttachedCollisionObject**: 模拟抓取物体附着到末端

### 参考代码核心流程

```cpp
// 1. 创建碰撞物体 (桌子、目标物体)
std::vector<moveit_msgs::CollisionObject> collision_objects;
// ... 设置物体形状和位置
planning_scene_interface.applyCollisionObjects(collision_objects);

// 2. 移动到抓取位置
geometry_msgs::Pose target_pose1;
// ... 设置位姿
group.setPoseTarget(target_pose1, "Link6");
group.plan(my_plan);
group.execute(my_plan);

// 3. 附加物体到末端 (模拟抓取)
moveit_msgs::AttachedCollisionObject attached_object;
attached_object.link_name = "Link6";
// ... 设置物体属性
planning_scene_interface.applyAttachedCollisionObject(attached_object);

// 4. 移动到放置位置
// ... 类似步骤2

// 5. 分离物体 (模拟放置)
planning_scene_interface.applyAttachedCollisionObject(detach_object);
```

## RealMan API 实现对比

### 技术栈差异

| 功能 | MoveIt/ROS | RealMan API |
|------|-----------|-------------|
| 运动规划 | `MoveGroupInterface.plan()` | 直接API调用,规划在控制器端 |
| 碰撞检测 | `PlanningSceneInterface` | `rm_set_collision_detection()` |
| 关节运动 | `setJointValueTarget()` | `rm_movej()` |
| 直线运动 | `setPoseTarget()` | `rm_movel()` |
| 夹爪控制 | GripperCommand action | `rm_set_gripper_pick/release` |
| 工具坐标系 | `end_effector_link` | `rm_set_tool_frame()` |

### API 映射

#### 运动控制

```python
# Moveit: 关节运动
group.setJointValueTarget(joint_values)
group.plan(my_plan)
group.execute(my_plan)

# RealMan API:
arm.rm_movej(joint_angles, speed, acceleration, 0)
```

```python
# MoveIt: 直线运动
group.setPoseTarget(pose, "Link6")
group.plan(my_plan)
group.execute(my_plan)

# RealMan API:
pose_list = [x, y, z, rx, ry, rz]  # 位置 + 欧拉角
arm.rm_movel(pose_list, speed, acceleration, 0)
```

#### 夹爪控制

```python
# MoveIt: 使用 GripperCommand action
# (需要配置夹爪的 URDF 和 controller)

# RealMan API:
arm.rm_set_gripper_pick_on()    # 闭合
arm.rm_set_gripper_release()    # 松开
```

#### 碰撞物体

MoveIt 使用 `CollisionObject` 进行环境建模,RealMan API 使用:

```python
# 设置碰撞检测
arm.rm_set_collision_detection(enable)
# 设置电子围栏
arm.rm_set_electronic_fence_config(config)
```

## 手眼标定

### 概念
手眼标定是确定相机坐标系与机器人坐标系之间变换矩阵的过程。

### 标定方法

#### 方法1: 使用标定板 (推荐)

1. 准备标定板 (如 ArUco 或 棋盘格)
2. 将标定板固定在机器人末端
3. 移动机器人到多个位姿,在每种位姿下:
   - 机器人记录末端位姿
   - 相机识别标定板并计算其相对于相机的位姿
4. 使用标定算法计算变换矩阵

#### 方法2: 点对应法

1. 在工作空间中选择多个特征点
2. 用相机测量的相机坐标
3. 用机器人移动到这些点,记录机器人坐标
4. 计算变换矩阵

### Python 标定示例

```python
import numpy as np
from scipy.spatial.transform import Rotation

def hand_eye_calibration(robot_points, camera_points):
    """
    手眼标定 - 计算相机到机器人的变换矩阵

    Args:
        robot_points: 机器人坐标系中的点 (N×3)
        camera_points: 相机坐标系中的点 (N×3)

    Returns:
        4×4 变换矩阵
    """
    # 使用 SVD 求解
    # ... (省略详细实现)

    return transformation_matrix
```

## 实现文件说明

### test_pick_place.py

主要的抓取放置演示程序,包含:

- **RM65Controller**: 机械臂控制封装
  - 连接/断开
  - 关节运动 (`movej`)
  - 直线运动 (`movel`)
  - 夹爪控制 (`gripper_pick/release`)

- **VisionSystem**: 视觉系统封装
  - D435 相机控制
  - 颜色检测
  - 深度获取

- **PickPlaceDemo**: 抓取放置演示
  - 完整的 pick-and-place 流程
  - 视觉引导模式

### 使用方法

```bash
# 激活环境
conda activate rm65

# 运行演示
python test_pick_place.py
```

## 与参考实现的区别

### 参考实现的特点

1. **仿真导向**: 使用 `AttachedCollisionObject` 模拟抓取
2. **规划可视化**: 可在 Rviz 中查看轨迹
3. **碰撞建模**: 显式定义环境物体进行碰撞检测

### 当前实现的特点

1. **硬件控制**: 直接控制 RM65 实体机器人
2. **视觉引导**: 使用 D435 实时检测物体
3. **简化碰撞**: 使用 API 级别的碰撞检测,不显式建模

### 实现真实抓取 (而非仿真)

参考代码中 `AttachedCollisionObject` 只是 MoveIt 的**可视化**功能,
告诉规划系统"这个物体现在附着在机器人上了,不要检测它与机器人的碰撞"。

真实抓取需要:
1. **物理夹爪**: 实际闭合/张开
2. **力控/反馈**: 确认是否成功抓取
3. **视觉验证**: 确认物体是否被拿起

## 下一步

1. **手眼标定**: 获取准确的相机-机器人变换
2. **夹爪配置**: 确认夹爪型号和控制参数
3. **安全测试**: 低速测试每个动作
4. **视觉调优**: 调整检测参数以适应实际物体

## 参考资料

- [MoveIt Tutorials](https://moveit.picknik.ai/tutorials/)
- [RealMan Robot API 文档](http://www.realman-robotics.com/)
- [RealSense D435 文档](https://dev.intelrealsense.com/docs/intel-realsense-d400-series)
