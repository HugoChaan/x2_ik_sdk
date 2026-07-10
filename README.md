# x2_ik_sdk

X2 上肢位置 IK SDK：输入当前 `arm_pos[14]` 和目标末端位置 `xyz`，输出新的 `arm_pos[14]`。

```text
X2 URDF + current arm_pos[14] + target xyz
-> single-arm position IK
-> UpperBodyCommandArray.arm_pos[14]
```

本仓库只提供：

1. 离线 IK 验证 CLI。
2. Python IK API。
3. 最小 ROS2 真机 adapter。

不包含视觉、抓取策略、手指规划，也不替代官方运控。

## 安装

```bash
cd x2_ik_sdk
python -m pip install --no-build-isolation -e .
```

依赖在 [pyproject.toml](pyproject.toml) 中声明：

```text
numpy
pin
```

## URDF

默认路径：

```text
third_party/urdf_for_shucai/x2_ultra_plus_omnipicker_omnipicker.urdf
```

使用前把 `urdf_with_oh:op.zip` 解压后的 `urdf_for_shucai` 放到 `third_party/` 下。

模型在其他位置时：

```bash
export X2_IK_URDF=/path/to/your_robot.urdf
```

## 离线验证

```bash
x2-ik-demo --side right --target-offset 0.01 0.00 0.01 --print-json
```

成功标准：

```text
"success": true
"message": "converged"
```

其他常用命令：

```bash
x2-ik-demo --side left --target-offset 0.01 0.00 0.01 --print-json
x2-ik-demo --limits
```

## Python API

```python
from x2_ik_sdk import ArmSide, X2ArmIKSolver, X2IKConfig

solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
current_arm_pos = solver.ready_arm_pos()

result = solver.solve_position(
    side=ArmSide.RIGHT,
    target_xyz=[0.32, -0.39, 0.23],
    current_arm_pos=current_arm_pos,
)

if result.success:
    arm_pos = result.arm_pos
```

`result.arm_pos` 长度为 14，可填入 `aimdk_msgs/msg/UpperBodyCommandArray.arm_pos`。

`arm_pos[14]` 顺序：

```text
0  left_shoulder_pitch_joint
1  left_shoulder_roll_joint
2  left_shoulder_yaw_joint
3  left_elbow_joint
4  left_wrist_yaw_joint
5  left_wrist_pitch_joint
6  left_wrist_roll_joint
7  right_shoulder_pitch_joint
8  right_shoulder_roll_joint
9  right_shoulder_yaw_joint
10 right_elbow_joint
11 right_wrist_yaw_joint
12 right_wrist_pitch_joint
13 right_wrist_roll_joint
```

## 真机最小流程

真机依赖官方 X2 SDK ROS2 环境：

```text
rclpy
aimdk_msgs
/mc/upper_body_command
/aimdk_5Fmsgs/srv/GetAllJointState
/aimdk_5Fmsgs/srv/SetMcAction
```

每次新开终端：

```bash
source ~/venv-x2ik/bin/activate
source ~/aimdk/install/setup.bash
export X2_IK_URDF=~/x2_ik_sdk/third_party/urdf_for_shucai/x2_ultra_plus_omnipicker_omnipicker.urdf
```

先 dry-run，只读关节、计算 IK、不发动作：

```bash
x2-ik-ros2-node \
  --side right \
  --target 0.32 -0.39 0.23 \
  --dry-run
```

看到 `IK success=True` 后，再切上肢外控：

```bash
python -m py_examples.set_mc_action SD
python -m py_examples.set_mc_action URS
```

小幅真实动作：

```bash
x2-ik-ros2-node \
  --side right \
  --target 0.32 -0.39 0.23 \
  --duration 6.0 \
  --skip-mode-switch
```

结束后恢复：

```bash
python -m py_examples.set_mc_action SD
```

安全约束：

```text
首次只测右臂
目标位移要小
duration 取 4-8 秒
现场有人看护急停
异常时立刻 Ctrl+C
```

## 常见问题

### `URDF not found`

检查 `urdf_with_oh:op.zip` 是否已解压、默认路径是否存在、`X2_IK_URDF` 是否正确。

### `No module named pinocchio`

```bash
python -m pip install numpy pin
```

### dry-run 成功但真机不动

优先检查是否已切到 `URS`，是否已 `source ~/aimdk/install/setup.bash`，以及 `/mc/upper_body_command`、`aimdk_msgs` 是否可用。

## 推荐顺序

```text
安装 SDK
-> 准备 URDF
-> 离线 CLI 验证
-> 真机 dry-run
-> SD
-> URS
-> 小幅真实动作
-> SD 恢复
```
