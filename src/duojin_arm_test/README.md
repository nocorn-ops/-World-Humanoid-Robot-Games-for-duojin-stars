# R1 Lite 机械臂笛卡尔控制链测试

该包验证工控机 `~/galaxea/install_430` 提供的控制链：

```text
/hdas/feedback_arm_left
  → Relaxed IK 当前末端位姿
  → 本节点增加小幅笛卡尔偏移
  → /motion_target/target_pose_arm_left
  → Relaxed IK
  → /motion_target/target_joint_state_arm_left
  → Joint Tracker
  → /motion_control/control_arm_left
  → HDAS
```

默认目标是左臂当前末端位姿沿 `base_link` Z 轴上移 `0.03 m`，姿态不变。

## 安全约束

- 默认是预览模式，不发布运动指令。
- 笛卡尔偏移默认最大为 8 cm。
- 没有当前末端反馈或 Relaxed IK 订阅者时拒绝执行。
- 永远不直接发布 `hdas_msg/MotorControl`。
- `r1lite_teleop` 运行时，执行脚本拒绝自主机械臂运动。

这些检查不能证明路径无碰撞。首次试验必须清空机械臂周围，一次只测试一条手臂，
操作员握住急停。

## 1. 工控机构建

```bash
cd ~/duojin_ws
./scripts/build_robot.sh
```

构建顺序为 `~/galaxea/install_430` underlay → `~/duojin_ws/install` overlay。

## 2. 启动完整厂商系统

```bash
cd ~/duojin_ws
./scripts/start_robot_sdk.sh
```

该命令配置 CAN 并通过 `R1LITEBody.d` 启动 HDAS、Joint Tracker、左右臂 Relaxed IK
等完整厂商系统。不要再手动启动第二份控制器。

## 3. 检查左臂控制链

```bash
cd ~/duojin_ws
./scripts/start_arm_environment.sh left
```

脚本只检查以下内容，不启动任何 SDK 节点：

- `/hdas/feedback_arm_left` 有实时反馈；
- `/r1_lite_jointTracker_demo_node` 已运行；
- `/relaxed_ik_left` 已运行；
- `/relaxed_ik/motion_control/pose_ee_arm_left` 有输出。

右臂把 `left` 改成 `right`。

## 4. 预览与执行

预览目标，机械臂不会运动：

```bash
./scripts/run_arm_ik_test.sh
```

执行前停止可能争夺目标话题的 Gello 遥操作 session：

```bash
tmux kill-session -t r1lite_teleop
./scripts/run_arm_ik_test.sh --ros-args -p execute:=true
```

右臂测试：

```bash
./scripts/start_arm_environment.sh right
./scripts/run_arm_ik_test.sh --ros-args \
  -p arm:=right -p execute:=true
```

修改偏移的示例：

```bash
./scripts/run_arm_ik_test.sh --ros-args \
  -p delta_x:=0.02 -p delta_z:=0.0 -p execute:=true
```

## 5. 逐级诊断

```bash
ros2 topic echo /relaxed_ik/motion_control/pose_ee_arm_left --once
ros2 topic echo /motion_target/target_pose_arm_left
ros2 topic echo /motion_target/target_joint_state_arm_left
ros2 topic echo /hdas/feedback_arm_left --once
```

末端目标存在但无关节目标时检查 Relaxed IK；关节目标存在但机械臂不动时检查
Joint Tracker、`/motion_control/control_arm_left` 和 HDAS 状态。
