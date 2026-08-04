# R1 Lite 机械臂笛卡尔链路预览诊断

该包只用于预览诊断工控机 `~/galaxea/install_430` 提供的笛卡尔链路。
当前允许的数据流在计算并打印目标后结束：

```text
/hdas/feedback_arm_left
  → Relaxed IK 当前末端位姿
  → 本节点增加小幅笛卡尔偏移
  → 打印 preview 目标
  → 结束，不发布 /motion_target/*
```

默认目标是把厂商 Relaxed IK 当前 FK 数值沿求解坐标系 `torso_link3` 的 Z 轴增加
`0.03 m`，姿态不变。该厂商诊断话题的 header 不能作为公共坐标语义；需要
`base_link` 绝对坐标和真实 TF 转换时，必须使用统一 API 的 `move_to`。

## 安全约束

- 该诊断只允许 preview，禁止传入 `execute:=true`。
- 笛卡尔偏移默认最大为 8 cm。
- 没有当前末端反馈或 Relaxed IK 订阅者时诊断失败。
- 只订阅反馈和读取 ROS 图，不创建或发布任何 `/motion_target/*` publisher。
- 永远不直接发布 `hdas_msg/MotorControl`。
- 比赛程序和新测试应使用统一机械臂 API，不应依赖本测试包。

厂商 Relaxed IK 在收到 Pose 时会直接向 Joint Tracker 发布关节目标，
这个副作用发生在项目能事前校验 IK 关节输出之前。因此本诊断节点保持 preview-only；
真实 Pose 运动必须使用统一 API，由它独占所有权、做事后 IK 检查并用 FK 闭环判定结果。

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

## 4. 仅预览诊断

预览左臂目标，机械臂不会运动：

```bash
./scripts/run_arm_ik_test.sh
```

右臂预览：

```bash
./scripts/start_arm_environment.sh right
./scripts/run_arm_ik_test.sh --ros-args -p arm:=right
```

修改偏移的示例：

```bash
./scripts/run_arm_ik_test.sh --ros-args \
  -p delta_x:=0.02 -p delta_z:=0.0
```

不得向任何上述命令增加 `-p execute:=true`。Pose 预览在比赛程序中应改用
统一 API 的 `move_to(..., execute=False)`。

真机路径先用 `./start.sh --enable-arm-motion` 打开 server 级许可，完成现场检查后，
通过统一 `move_to`、`move_by` 或 `move_joints` 的 `execute=True` 执行。完整步骤见
[`../../docs/runbooks/arm-motion-api.md`](../../docs/runbooks/arm-motion-api.md)。

## 5. 逐级诊断

```bash
ros2 topic echo /relaxed_ik/motion_control/pose_ee_arm_left --once
ros2 topic echo /hdas/feedback_arm_left --once
ros2 topic info /motion_target/target_pose_arm_left -v
ros2 topic info /motion_target/target_joint_state_arm_left -v
```

这些命令只读取反馈和 ROS 图端点。运行 `run_arm_ik_test.sh` preview 时，
不应由该脚本产生新的 `/motion_target/target_pose_arm_*` 或
`/motion_target/target_joint_state_arm_*` 消息。
