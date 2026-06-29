# RM65-B


### 5. 相关脚本

| 脚本 | 用途 |
|------|------|
| `src/examples/get_gripper_status.py` | 查询夹爪/末端完整状态（推荐） |
| `src/examples/get_arm_state.py` | 获取机械臂关节/位姿/错误码 |
| `src/examples/get_arm_state_to_md.py` | 把关节/位姿/错误码写入md文件 |
| `src/examples/move_to_zero.py` | 机械臂位姿归零 |
| `src/examples/test_trajectory.py` | 机械臂轨迹记录和回放 |
| `src/examples/test_gripper_cycle.py` | 夹爪循环/耐久测试（`循环次数 间隔秒`）|


## 环境依赖

### Python环境
- **环境名称**: `rm65` (miniforge3)
- **Python版本**: 3.10.20
- **激活命令**: `conda activate rm65`

