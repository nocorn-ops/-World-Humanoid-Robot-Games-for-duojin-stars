# R1 Lite 机械臂笛卡尔控制链路测试

这个 ROS 2 包用来验证本地 `install_430` 中查到、但实际由工控机
`/home/r1lite/galaxea/install` 提供的控制链路：

```text
current arm feedback
  -> Relaxed IK forward kinematics/current EE pose
  -> this node adds a 3 cm Cartesian Z offset
  -> /motion_target/target_pose_arm_left
  -> Relaxed IK
  -> /motion_target/target_joint_state_arm_left
  -> Joint Tracker
  -> /motion_control/control_arm_left
  -> HDAS
```

默认目标是左臂当前末端位姿沿 `base_link` Z 轴向上平移 `0.03 m`，末端
姿态不变。这比写死一个可能不可达的绝对坐标更适合首次真机验证。

## 安全行为

- 默认为预览模式，不发布运动指令。
- 笛卡尔偏移默认最大为 8 cm。
- 没有当前末端反馈或 Relaxed IK 订阅者时拒绝执行。
- 永远不直接发布 `hdas_msg/MotorControl`。

这些检查不能证明中间路径一定无碰撞。首次试验必须清空机械臂周围，使用
`fast_mode:=false`，操作员握住急停，且一次只测一条手臂。

## 1. 下载到工控机

项目推送到 GitHub、Gitee 或自建 Git 服务后，在工控机执行：

```bash
cd /home/r1lite
git clone <GIT_REPOSITORY_URL> duojin_ws
```

`.gitignore` 会确保仓库中不包含开发机生成的 `build/`、`install/`、`log/`。

## 2. 在工控机构建

```bash
cd /home/r1lite/duojin_ws
./scripts/build_robot.sh
```

构建时的 overlay 顺序固定为：

```text
/opt/ros/humble
  → /home/r1lite/galaxea/install
  → /home/r1lite/duojin_ws/install
```

## 3. 启动厂商控制链

先检查自动启动系统是否已经运行了 Joint Tracker。如果已经有同名节点，不要再启
第二份控制器。

```bash
source /opt/ros/humble/setup.bash
source /home/r1lite/galaxea/install/setup.bash
ros2 node list | grep -E 'jointTracker|relaxed_ik'
```

如果没有运行，分别在两个终端启动 R1 Lite 节点：

```bash
# 终端 1：关节轨迹跟踪与 MotorControl 输出
source /opt/ros/humble/setup.bash
source /home/r1lite/galaxea/install/setup.bash
ros2 launch mobiman r1_lite_jointTrackerdemo_launch.py fast_mode:=false

# 终端 2：左臂笛卡尔逆运动学
source /opt/ros/humble/setup.bash
source /home/r1lite/galaxea/install/setup.bash
ros2 launch mobiman r1_lite_left_arm_relaxed_ik_launch.py
```

该 Joint Tracker 会把 `/opt/galaxea/body/hardware.json` 校验为 `R1-LITE`。

## 4. 预览目标（机械臂不动）

```bash
cd /home/r1lite/duojin_ws
./scripts/run_arm_ik_test.sh
```

日志应显示当前末端坐标和 Z 值增加 0.03 m 后的目标，但机械臂不应运动。

## 5. 执行 3 cm 运动

检查打印的目标点、清空运动区域并握住急停后执行：

```bash
cd /home/r1lite/duojin_ws
./scripts/run_arm_ik_test.sh --ros-args -p execute:=true
```

如果改测右臂，先启动右臂 IK，再传入 `arm:=right`：

```bash
ros2 launch mobiman r1_lite_right_arm_relaxed_ik_launch.py
./scripts/run_arm_ik_test.sh --ros-args \
  -p arm:=right -p execute:=true
```

偏移量可以在 `max_delta_m` 限制内修改。例如让左臂末端沿 `base_link` X
轴前进 2 cm：

```bash
./scripts/run_arm_ik_test.sh --ros-args \
  -p delta_x:=0.02 -p delta_z:=0.0 -p execute:=true
```

## 6. 逐级验证

```bash
# Relaxed IK/FK 计算的当前末端位姿
ros2 topic echo /relaxed_ik/motion_control/pose_ee_arm_left --once

# 本测试包发布的末端位姿目标
ros2 topic echo /motion_target/target_pose_arm_left

# Relaxed IK 生成的关节目标
ros2 topic echo /motion_target/target_joint_state_arm_left

# HDAS 发布的真实关节反馈
ros2 topic echo /hdas/feedback_arm_left
```

如果末端目标已发布，但没有关节目标，检查 Relaxed IK 进程。如果已经有关节
目标但机械臂不动，检查 Joint Tracker、`/motion_control/control_arm_left`
和 HDAS 状态 Topic。
