# R1 Lite 机械臂 SDK 接口证据

本文是 `duojin_robot_interface` 使用机械臂厂商接口的事实来源。状态含义：

- `documented`：由同版本 `install_430` 源码、配置或二进制接口确认，尚未在本机 ROS 图观测。
- `observed`：已在目标机器 ROS 图/消息上观测。
- `robot-verified`：已通过受控真机运动验证。

截至 2026-07-29，本页新增结论均为 `documented`，不得写成已经真机验证。

## 控制与反馈接口

| 接口 | 方向/类型 | 已确认字段语义 | QoS/频率 | 状态与证据 |
| --- | --- | --- | --- | --- |
| `/motion_target/target_joint_state_arm_left`、`right` | 赛队/Relaxed IK → Joint Tracker；`sensor_msgs/msg/JointState` | `position[0:6]` 是 joint1..joint6 目标 rad；`velocity[0:6]` 是逐关节正的最大速度 rad/s；不按 `name` 重排 | tracker 订阅为 BEST_EFFORT、VOLATILE、KEEP_LAST 10；厂商示例发布为 RELIABLE、TRANSIENT_LOCAL、depth 10 | documented；SDK `r1_lite_test_open_box.py` 和 Joint Tracker 二进制 |
| `/motion_target/target_pose_arm_left`、`right` | 赛队 → Relaxed IK；`geometry_msgs/msg/PoseStamped` | xyz/quaternion 被直接取数；输入 `header.frame_id` 和 stamp 不参与 TF | Relaxed IK 订阅为 BEST_EFFORT、VOLATILE、KEEP_LAST 1 | documented；`mobiman/lib/mobiman/relaxed_ik.py` |
| `/hdas/feedback_arm_left`、`right` | HDAS → 赛队/IK；`sensor_msgs/msg/JointState` | 闭环可依赖 `position[0:6]`，顺序 joint1..joint6；本地快照不能证明 name/header/velocity/effort 始终有效 | IK 订阅为 BEST_EFFORT、VOLATILE、KEEP_LAST 1；诊断配置期望 200 Hz | documented；`rds_ros/topic_list.toml`、diagnosis config、Relaxed IK |
| `/motion_control/pose_ee_arm_left`、`right` | 全身 FK → 赛队；`geometry_msgs/msg/PoseStamped` | 由完整 R1 Lite URDF 和关节反馈计算左右 gripper pose | header frame、QoS、实际频率待 L3 记录 | documented；`r1_lite_eepose_launch.py`、SDK 启动配置 |

赛队程序只能向 `/motion_target/*` 发布。`/motion_control/control_arm_*` 的
`hdas_msg/msg/MotorControl` 是 Mobiman/Joint Tracker 到 HDAS 的内部链路，禁止直接发布。

## 六关节顺序与有效限位

两个手臂都按 base 到 gripper 的 `joint1 ... joint6` 顺序传数组。完整 URDF 的几何限位
略宽，但当前 Joint Tracker 实际会夹紧到下表。生产适配层按更窄的控制器有效限位再留
0.03 rad margin，并在目标越界时拒绝，不做静默裁剪。

| index | 关节 | URDF [rad] | Joint Tracker 有效范围 [rad] | API 初始可用范围 [rad] |
| --- | --- | --- | --- | --- |
| 0 | joint1 | [-2.8798, 2.8798] | [-2.8, 2.8] | [-2.77, 2.77] |
| 1 | joint2 | [0, 3.1416] | [0, 3.14] | [0.03, 3.11] |
| 2 | joint3 | [-3.3161, 0] | [-3.14, 0] | [-3.11, -0.03] |
| 3 | joint4 | [-1.5708, 1.5708] | [-1.57, 1.57] | [-1.54, 1.54] |
| 4 | joint5 | [-1.5708, 1.5708] | [-1.57, 1.57] | [-1.54, 1.54] |
| 5 | joint6 | [-2.8798, 2.8798] | [-2.8, 2.8] | [-2.77, 2.77] |

Joint Tracker 回调会直接读取前六个元素；短数组存在底层越界风险。因此适配层必须保证
`position` 和 `velocity` 都恰好是 6 个有限数，且每个 velocity 大于 0。

## Relaxed IK 的真实坐标语义

R1 Lite 左右设置均声明：

```text
base link: torso_link3
end link:  left_gripper_link / right_gripper_link
```

`relaxed_ik.py` 的 pose callback 忽略输入 header，直接把 xyz/quaternion 数值送进这个
KDL/Relaxed IK 链。因此：

```text
客户端目标（例如 base_link）
  → duojin_robot_interface 查 TF
  → 转成 torso_link3 下的数值
  → /motion_target/target_pose_arm_*
```

不能仅把输出消息的 `header.frame_id` 写成 `base_link` 就认为完成了转换。厂商 Relaxed IK
自己的 target/IK echo 会错标 `base_link`，current FK 又会错标 `left_ee/right_ee`；这些
话题只能用于诊断数值，不能作为本 API 的公共坐标语义。API 使用全身 FK pose 反馈，且
在 L3 首次运行时强制记录/核对其真实 header 与 TF。

项目公开 `MoveArmPose` Action 的 v1 时间语义是“使用最新可用 TF”。适配层不使用
Goal `target_pose.header.stamp` 做时间同步；Python `move_to` 生成零 stamp。这是当前项目
实现约束，不是厂商能力保证。将感知 Pose 接入该 Action 前，上层必须独立校验
感知结果的新鲜度；不得依赖 Goal stamp 让服务端自动拒绝历史目标。

`MoveArmRelative` 与上述绝对位置 Action 共用同一执行链。`delta.header.frame_id`
是增量三个分量所属坐标系；服务端先把当前末端位姿转到该 frame，再将
`delta.vector` 加到 xyz，朝向保持为校验时的当前朝向。默认 frame 由 Python
封装填为 `base_link`；原生 ROS Goal 的 frame 不得为空。

## 项目公开末端位姿话题

| 接口 | 方向/类型 | Frame/时间/QoS | 状态 |
| --- | --- | --- | --- |
| `/duojin/arm/left/current_pose`、`right` | API → 比赛程序；`geometry_msgs/msg/PoseStamped` | 默认 `base_link`；header stamp 是 API 发布时间；RELIABLE、VOLATILE、KEEP_LAST 1 | 项目契约 documented；真机数值/频率待 L3 |

API 默认每 `0.05 s` 检查一次。只有厂商 FK pose 和关节反馈均不超过
`0.10 s`、pose 样本确实更新、TF 可用且数值/四元数有效时才发布。它不会用
上一帧冒充实时数据；因此消费端必须对“收到新消息”设超时，不能仅检查话题存在。
该话题为纯只读适配，不会发布任何 `/motion_target/*`。

Relaxed IK 的 obstacle 列表为空，Joint Tracker task 中 self-collision 和 joint-position
limit 约束也未启用。数值可解不代表无碰撞，当前 API 也不提供碰撞规划。

## 速度、误差与新鲜度

证据值：

- SDK controller config 给出 R1 A1 X 速度 `[3, 3, 3, 5, 5, 5] rad/s`，fast 配置更高。
- 厂商 R1 Lite 开箱示例传入 `[1.6, 1.6, 1.6, 4, 4, 4]` 或更高的速度限制。
- Joint Tracker task 文件含六轴 `±0.5 rad/s` 的 MPC 约束，但这不是硬件硬上限。

这些都不适合作为比赛首次动作速度。API 初始采用以下**待真机标定的工程值**：

- 配置速度上限：每轴 0.25 rad/s，再乘 goal 的 `(0, 1]` speed scale；
- 关节到位：最大误差 0.02 rad，持续至少 0.25 s；
- 末端到位：位置 0.015 m、朝向 0.05 rad，持续至少 0.25 s；
- 反馈新鲜度：monotonic 接收间隔不超过 0.10 s；
- 单次关节变化：每轴不超过 0.35 rad；单次末端直线距离不超过 0.08 m。

这些限制不证明轨迹碰撞安全。Pose → Relaxed IK 路径只产生 joint position，不携带
velocity，而且厂商节点会在项目能够校验该结果前直接将它发给 Joint Tracker。
当 server 和 Goal 两道 execute 门都打开时，API 会显式允许这条实验性路径，然后检查新 IK
输出的关节限位/变化量并用末端 FK 闭环等待到位。这些检查是事后监控，不是发布前隔离。

## 零点、bias 与冷启动限制

HDAS 支持从机器人 `/opt/galaxea/body/hardware.json` 读取左右臂 `joint_bias`，标准
launch 的 `calib_enable` 默认却是 `disable`，bias limit 为 0.087 rad。标准启动脚本没有
显式机械 homing 步骤，但这不足以证明驱动内部必然使用永久绝对编码器。

在完成以下工控机/真机检查前，不得在项目文档中声称“底层一直知道绝对零点”：

```bash
ros2 param get /HDAS calib_enable
sudo sed -n '/joint_bias/,+20p' /opt/galaxea/body/hardware.json
ros2 topic echo /hdas/feedback_arm_left --once
ros2 topic echo /hdas/feedback_arm_right --once
```

还需用测试记录完成断电冷启动前后的重复关节角与 FK 对比。读取机器人专属配置可能需要
运维权限；不得由比赛程序自动修改它。
