# R1 Lite 底盘与躯干二开接口

本页根据官方运动控制/驱动接口资料和 2026-07-26 真机联调记录整理。面向二开程序
统一采用标准 ROS 2 消息；`/motion_control/*` 是 Mobiman 到 HDAS 的内部接口，赛队程序
不得直接发布 `hdas_msg/msg/MotorControl`。

## 赛队程序发送

| 话题 | 消息类型 | 说明 |
| --- | --- | --- |
| `/motion_target/target_speed_chassis` | `geometry_msgs/msg/TwistStamped` | `linear.x/y` 底盘线速度，`angular.z` 角速度 |
| `/motion_target/chassis_acc_limit` | `geometry_msgs/msg/TwistStamped` | 底盘线加速度和角加速度限制 |
| `/motion_target/target_joint_state_torso` | `sensor_msgs/msg/JointState` | 三个躯干关节的目标位置和最大速度 |
| `/motion_target/brake_mode` | `std_msgs/msg/Bool` | `true` 制动，`false` 解除 |

底盘速度范围：`linear.x/y` 为 `(-1.5, 1.5) m/s`，`angular.z` 为
`(-3, 3) rad/s`。速度命令必须持续发布，联调已验证 10 Hz 可用；单次发布不足以持续
驱动底盘。运动计时使用 `time.sleep(0.1)`，不要把收到反馈后会提前返回的
`spin_once(timeout_sec=...)` 当作定时器。

躯干目标的 `position` 与 `velocity` 均为 3 个元素。已验证发送裸字段即可，无需手动
填写 header。底盘、躯干联调使用默认 QoS 即可。

## 赛队程序接收

| 话题 | 消息类型 | 说明 |
| --- | --- | --- |
| `/hdas/feedback_chassis` | `sensor_msgs/msg/JointState` | 三个车轮角度、线速度和角速度 |
| `/hdas/feedback_status_chassis` | `hdas_msg/msg/FeedbackStatus` | 底盘错误状态 |
| `/hdas/feedback_torso` | `sensor_msgs/msg/JointState` | 三个躯干关节的位置、速度和力矩 |
| `/hdas/feedback_status_torso` | `hdas_msg/msg/FeedbackStatus` | 躯干错误状态 |
| `/hdas/imu_chassis` | `sensor_msgs/msg/Imu` | 底盘姿态、角速度和线加速度 |
| `/hdas/imu_torso` | `sensor_msgs/msg/Imu` | 躯干姿态、角速度和线加速度 |

`/hdas/feedback_chassis.velocity[0:3]` 是三个车轮线速度，`velocity[3:6]` 是角速度。
`/hdas/feedback_torso` 的 `position/velocity/effort[0:3]` 对应三个关节，第 4 个元素
恒为 0。

## 启动与验证

完整 SDK 启动后至少检查：

```bash
source ~/galaxea/install_430/setup.bash
ros2 topic list | grep -E 'motion_target|hdas/feedback|motion_control'
ros2 topic echo /hdas/feedback_chassis --once
ros2 topic echo /hdas/feedback_torso --once
```

话题 I/O 方向在官方“运动控制”和“驱动”页面中采用了不同组件视角。二开时只需遵循：
赛队程序向 `/motion_target/*` 发布，从 `/hdas/*` 接收，不直接操作
`/motion_control/*`。
