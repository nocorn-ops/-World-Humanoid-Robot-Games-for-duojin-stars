# R1 Lite 机械臂统一 API 使用手册

本手册面向 R1 Lite 工控机上的比赛程序和真机操作员。所有命令中的 `~`
均指工控机用户 `r1lite` 的 `/home/r1lite`，不是本地开发机路径。

实现结果是：无参数 `./start.sh` 完成 SDK 启动和整机检查后，会自动在
`duojin_arm_api` tmux 会话启动唯一的 preview API，再进入已经 source 好的
`[duojin]` Shell。此时程序可以直接调用，但默认不会发布运动目标。

## 1. 能力和边界

左右臂各提供三种目标：

- `move_to`：绝对末端 `(x, y, z)`，单位 m；Python 默认 `base_link`，保持当前朝向。
- `move_by`：在指定 frame 三根轴上增加 `(dx, dy, dz)`，单位 m；保持当前朝向。
- `move_joints`：将 joint1 到 joint6 移动到六个绝对关节角，单位为 rad。

左右臂可独立调用，但同一条手臂同时只执行一个目标。第二个同臂目标不会抢占，
而是返回 `BUSY`；如需更换目标，先 cancel 原目标并等待它返回。

这是带反馈到位判定的目标 API，不是轨迹规划器。它当前不提供：

- 碰撞规划、自碰检测、障碍物避让或奇异点保证；
- 双臂同步轨迹、躯干/底盘/夹爪联动或力控；
- 修改末端朝向；
- 机械零点标定、HDAS bias 修改或自动 homing。

数值合法、IK 有解都不等于运动路径无碰撞。取消、超时或异常时的软件 hold 只是
best-effort 停止手段，**不是硬件急停**。

厂商 Relaxed IK 收到 Pose 后会先于本项目的 IK 输出校验，直接向 Joint Tracker 发布
关节目标。统一 API 在双重 execute 许可下允许这条实验性路径，然后事后检查新 IK
输出的关节限位/变化量，并使用 FK 末端反馈闭环等待到位。旧诊断脚本仍保持 preview-only。

## 2. 首次使用和源码变更后编译

以下操作只在 R1 Lite 工控机执行。首次使用，或者任何 `src/` 中的 Python、launch、
Action 定义、package 依赖发生变更后，必须重新编译 overlay：

```bash
cd ~/duojin_ws
./scripts/build_robot.sh
```

只修改 Markdown 文档或项目 Shell 脚本时不需要 `colcon build`。本地开发机没有
ROS 2 Humble 是已知条件；L2 构建必须在工控机完成，不得把本地无法构建冒充为
已验证。

编译后可确认 Action 定义已生成：

```bash
source ~/galaxea/install_430/setup.bash
source ~/duojin_ws/install/setup.bash
ros2 interface show duojin_interfaces/action/MoveArmPose
ros2 interface show duojin_interfaces/action/MoveArmRelative
ros2 interface show duojin_interfaces/action/MoveArmJoints
```

## 3. 每次真机启动前的强制条件

启动 SDK 前完成下列检查：

1. 操作员在机器人旁，全程握住可用的硬件急停。
2. 机器人周围已清场，当前手臂和目标路径无人、无货架、无桌面或线缆干涉。
3. 底盘已停止并制动。未停止底盘时不得运行本 API 的 execute 模式。
4. 机器人供电、CAN 和 SDK 状态正常；当前左右关节反馈和末端 FK 与实际姿态相符。
5. `r1lite_teleop`、Gello、VR、诊断脚本和其他自主节点不再发布同一机械臂目标。
6. SDK、Joint Tracker、Relaxed IK 和本 API server 均只运行一份。不要通过重复启动
   同名节点“修复”链路。
7. 所有客户端、server 与 SDK 使用相同的 `ROS_DOMAIN_ID`。

SDK 标准启动链没有已确认的显式机械 homing 步骤，这不能推导为“底层永远知道
绝对零点”。首次 L4 或机器人拆装/标定/断电后，先对照实际姿态核对两臂的
`/hdas/feedback_arm_*` 和 `/relaxed_ik/motion_control/pose_ee_arm_*`。不得由比赛程序自动修改
`/opt/galaxea/body/hardware.json` 或 joint bias。零点证据和待验证项见
[`docs/interfaces/arm.md`](../interfaces/arm.md)。

## 4. 启动 SDK 和 API server

### 4.1 启动整机环境

在首个工控机终端输入队伍实际 domain 整数，然后启动：

```bash
read -r -p "ROS_DOMAIN_ID: " ROS_DOMAIN_ID
export ROS_DOMAIN_ID
cd ~/duojin_ws
./start.sh
```

`start.sh` 会启动完整厂商 SDK、关闭 `r1lite_teleop`、检查整机链路，成功后进入
`[duojin]` Shell，并自动启动 preview API。任何 `[FAIL]` 都必须先排除；不得跳过检查
直接运动，也不要手工补启单个厂商控制节点。

### 4.2 预览模式（默认）

无参数启动已经创建 preview server，无需再运行 `ros2 launch`。直接确认六个端点和两个位姿话题：

```bash
ros2 action list -t | grep '^/duojin/arm/'
ros2 topic list -t | grep '/duojin/arm/.*/current_pose'
```

此模式的 server 级 `execute=false` 是强制门。goal 使用默认的 `execute=false` 会完成预览；
即使 goal 显式请求 `execute=true` 也不会发布目标，而会返回
`RETRYABLE_FAILURE/EXECUTION_DISABLED`。

### 4.3 真机执行模式

先用 `./stop.sh` 完整停止 preview 环境，确认没有活动 goal，再重新完成第 3 节的安全
检查。只在操作员明确授权后从停止状态启动：

```bash
./start.sh --enable-arm-motion
```

此模式强制要求双臂 Pose/关节目标订阅者、双臂 HDAS 执行链、关节反馈和
双臂末端位姿全部通过。固定节点名只是诊断信息；每个具体 Goal 的 API 门禁还会
核对预期 SDK 订阅端点。底盘、躯干、夹爪、IMU、BMS
和相机仍会检查，但失败显示 `[WARN]` 而不阻止手动机械臂 API；
这些 WARN 仍表示整机比赛环境未就绪。

运动只在两道门都打开时发生：

| server `execute` | goal `execute` | 结果 |
| --- | --- | --- |
| `false` | `false` | 完整预览，`SUCCESS/PREVIEW_COMPLETE`、`executed=false` |
| `false` | `true` | 拒绝执行，`RETRYABLE_FAILURE/EXECUTION_DISABLED`、`executed=false` |
| `true` | `false` | 完整预览，`SUCCESS/PREVIEW_COMPLETE`、`executed=false` |
| `true` | `true` | `move_to`、`move_by` 和 `move_joints` 通过对应门禁后真实执行 |

不得同时保留 preview server 和 execute server，也不得用两个 execute server。正向启动参数
只打开 server 门；Python/Goal 默认仍是 `execute=false`。

### 4.4 新终端的 source 顺序

`start.sh` 只会为它打开的 ready Shell 加载环境。所有新终端都必须设置同一 domain，
并严格先 source SDK underlay，再 source 项目 overlay：

```bash
read -r -p "same ROS_DOMAIN_ID used by start.sh: " ROS_DOMAIN_ID
export ROS_DOMAIN_ID
source ~/galaxea/install_430/setup.bash
source ~/duojin_ws/install/setup.bash
```

确认六个端点、两个位姿话题和类型：

```bash
ros2 action list -t | grep '^/duojin/arm/'
ros2 action info /duojin/arm/left/move_to
ros2 topic info /duojin/arm/left/current_pose
```

`ros2 action info` 中每个端点应只有一个 Action server；为零表示 server 未启动，多于一个必须
先停掉重复 server。

## 5. ROS 契约

### 5.1 Action 端点

| 手臂 | 绝对末端 | 相对末端 | 六关节 |
| --- | --- | --- | --- |
| 左臂 | `/duojin/arm/left/move_to`<br>`MoveArmPose` | `/duojin/arm/left/move_by`<br>`MoveArmRelative` | `/duojin/arm/left/move_joints`<br>`MoveArmJoints` |
| 右臂 | `/duojin/arm/right/move_to`<br>`MoveArmPose` | `/duojin/arm/right/move_by`<br>`MoveArmRelative` | `/duojin/arm/right/move_joints`<br>`MoveArmJoints` |

两侧还发布 `geometry_msgs/msg/PoseStamped` 的
`/duojin/arm/{left,right}/current_pose`，默认已转到 `base_link`。

### 5.2 `MoveArmPose.Goal`

```text
geometry_msgs/PoseStamped target_pose
bool keep_current_orientation
float64 timeout_sec
bool execute
```

- `target_pose.header.frame_id`：公共接口推荐显式填 `base_link`。服务端会在发给 IK 前做
  TF 数值转换；Python 默认是 `base_link`，原生 ROS Goal 不会自动补默认值，
  `frame_id` 为空会返回 `INVALID_GOAL`。不得通过只改 frame 名字伪造转换。
- `target_pose.header.stamp`：v1 会将 Goal 按“最新可用 TF”转换，不使用客户端
  stamp 做时间同步。Python 和本手册的 CLI 示例都保持零 stamp。不得把未经
  上层新鲜度校验的历史感知 Pose 直接作为运动目标。
- `target_pose.pose.position`：绝对 x/y/z，单位 m。
- `target_pose.pose.orientation`：本版不使用客户端朝向，但消息应填有效单位四元数。
- `keep_current_orientation`：本版必须为 `true`；`false` 会被拒绝。
- `timeout_sec`：`0.0` 表示由 server 根据配置/目标选择超时；正数表示本 goal 超时。
- `execute`：默认 `false`，只预览。server 也以 `execute=true` 启动时，Pose 会真实发给 IK。

### 5.3 `MoveArmRelative.Goal`

```text
geometry_msgs/Vector3Stamped delta
bool keep_current_orientation
float64 timeout_sec
bool execute
```

`delta.vector` 是在 `delta.header.frame_id` 三根轴上的 xyz 增量，单位 m。Python
默认补为 `base_link`；原生 ROS Goal 必须填非空 frame。服务端先取新鲜的
当前位姿再计算绝对目标，所以它是“调用时刻”的相对移动，并且保持当前朝向。
`keep_current_orientation` 必须为 `true`。物理执行与 `move_to` 使用同一双重许可和闭环。

### 5.4 `MoveArmJoints.Goal`

```text
float64[6] positions_rad
float64 speed_scale
float64 timeout_sec
bool execute
```

- `positions_rad`：joint1 到 joint6 的六个绝对角，恰好 6 项，全部有限且不超过配置限位。
- `speed_scale`：范围 `(0, 1]`，乘以 server 的逐关节速度上限。Python 默认 `0.2`。
- `timeout_sec`：`0.0` 表示 server 自动选择；正数是本 goal 超时。
- `execute`：与末端 Action 相同，省略或 `false` 都只 preview。

### 5.5 Result 和 Feedback

`MoveArmPose.Result` 与 `MoveArmRelative.Result` 都返回 `status`、`final_pose`、`final_pose_valid`、`position_error_m` 和
`orientation_error_rad`；运行中 Feedback 返回 `phase`、`current_pose`、两种误差与
`elapsed_sec`。通过初步目标与反馈校验后，有效的 `final_pose` 和
`current_pose` 使用本次 Goal 的命令 frame（绝对目标的 `target_pose.header.frame_id`
或相对目标的 `delta.header.frame_id`）；`BUSY`、目标无效、
SDK 未就绪等早期失败以及最初的 `VALIDATING` Feedback 可能不含有效 Pose；此时
`final_pose_valid=false`，不得读取占位 Pose 或误差。
`MoveArmJoints.Result` 返回 `status`、
`final_positions_rad[6]`、`final_positions_valid` 和
`max_position_error_rad`；运行中 Feedback 返回 `phase`、`current_positions_rad[6]`、最大误差与
`elapsed_sec`。`final_positions_valid=false` 时六个零和误差只是 ROS 定长字段占位值。

`status` 是 `duojin_interfaces/msg/ArmMotionStatus`，含 `outcome`、`reason`、`message` 和
`executed`。只有最终 result 能表示到位或失败；“goal accepted”、`PUBLISHING` Feedback 或
“已发话题”都不等于机械臂到位。需要生成式类型的完整字段时使用：

```bash
ros2 interface show duojin_interfaces/action/MoveArmPose
ros2 interface show duojin_interfaces/action/MoveArmRelative
ros2 interface show duojin_interfaces/action/MoveArmJoints
ros2 interface show duojin_interfaces/msg/ArmMotionStatus
```

## 6. Python 推荐用法

### 6.1 可复用的 `ArmClient`

比赛程序中推荐每条手臂复用一个 `ArmClient`，而不是每个小步骤重新初始化 ROS。
下例的 `target_x_m`/`target_y_m`/`target_z_m` 必须是已经用 TF 和环境检查确认的
`base_link` 绝对目标，不是相对位移：

```python
from duojin_robot_interface.arm_client import ArmClient
from duojin_interfaces.msg import ArmMotionStatus

target_x_m = ...
target_y_m = ...
target_z_m = ...

with ArmClient("left", server_timeout_sec=5.0) as arm:
    preview = arm.move_to(
        target_x_m,
        target_y_m,
        target_z_m,
        frame_id="base_link",
        timeout_sec=0.0,
        execute=False,
    )
    if (
        not preview.succeeded
        or preview.reason != ArmMotionStatus.PREVIEW_COMPLETE
        or preview.executed
    ):
        raise RuntimeError(f"arm preview failed: {preview.reason}: {preview.message}")

```

`move_to` 在 server 校验目标时读取最新末端朝向并保持它；客户端不需要传四元数。
在 `./start.sh --enable-arm-motion` 环境内，人工审核 preview 后可对同一目标传
`execute=True`。调用会发布 Pose，等待 IK 新输出，再用 FK 末端反馈判定到位。

查询当前坐标和相对 preview：

```python
from duojin_robot_interface import ArmClient

with ArmClient("left") as arm:
    pose = arm.get_pose(timeout_sec=2.0, max_age_sec=0.25)
    print(pose.frame_id, pose.position_xyz, pose.orientation_xyzw, pose.stamp_sec)

    preview = arm.move_by(
        0.01, 0.0, 0.0,
        frame_id="base_link",
        timeout_sec=0.0,
        execute=False,
    )
    if not preview.succeeded:
        raise RuntimeError(f"relative preview failed: {preview.reason}: {preview.message}")

    # 仅在 ./start.sh --enable-arm-motion 启动且 preview 已人工审核时：
    result = arm.move_by(
        0.01, 0.0, 0.0, frame_id="base_link", execute=True
    )
    if not result.succeeded:
        raise RuntimeError(f"relative motion failed: {result.reason}: {result.message}")
```

`get_pose()` 等待一条本客户端收到时间不超过 `max_age_sec` 的新鲜消息；超时会抛
`TimeoutError`，不返回旧坐标。`move_by` 默认增量沿 `base_link` 的轴；如果显式
传末端 frame，增量就沿调用时的末端自身轴。frame 名必须以当前真机 TF 为准，
不得猜测为 `left_ee` 之类的名字。

关节调用示例：

```python
from duojin_robot_interface.arm_client import ArmClient

target_positions_rad = [q1, q2, q3, q4, q5, q6]

with ArmClient("right") as arm:
    preview = arm.move_joints(
        target_positions_rad,
        speed_scale=0.2,
        timeout_sec=0.0,
        execute=False,
    )
    if not preview.succeeded:
        raise RuntimeError(f"joint preview failed: {preview.reason}: {preview.message}")

    # 仅当本环境由 ./start.sh --enable-arm-motion 启动且现场条件仍安全时：
    result = arm.move_joints(
        target_positions_rad,
        speed_scale=0.2,
        timeout_sec=0.0,
        execute=True,
    )
    if not result.succeeded:
        raise RuntimeError(f"joint motion failed: {result.reason}: {result.message}")
```

### 6.2 单次模块函数

简单脚本可使用模块函数；`arm` 被设计为必须显式填写的 keyword-only 参数，
防止默认选错手臂：

```python
from duojin_robot_interface import get_pose, move_by, move_joints, move_to

pose_result = move_to(
    target_x_m,
    target_y_m,
    target_z_m,
    arm="left",
    frame_id="base_link",
    timeout_sec=0.0,
    execute=False,
)

joint_result = move_joints(
    [q1, q2, q3, q4, q5, q6],
    arm="right",
    speed_scale=0.2,
    timeout_sec=0.0,
    execute=False,
)

pose = get_pose(arm="left")
relative_preview = move_by(
    0.01, 0.0, 0.0, arm="left", frame_id="base_link", execute=False
)
```

### 6.3 结果和取消

`ArmResult` 包含：

```text
succeeded
outcome
reason
message
executed
execution_state_known
final_state_valid
final_position_xyz
final_orientation_xyzw
final_frame_id
final_positions_rad
position_error_m
orientation_error_rad
max_position_error_rad
```

`outcome` 和 `reason` 是整数，应与 `duojin_interfaces.msg.ArmMotionStatus` 中的命名
常量比较，不要与日志文字比较。预览通过时是 `succeeded=True`、
`executed=False`、`reason == ArmMotionStatus.PREVIEW_COMPLETE`。真机
Action 被 server 接受不等于已到位；必须检查最终 `succeeded`、`reason` 和误差字段。
最终位姿/关节值和误差只在 `final_state_valid=True` 时可用。

如果目标发送后客户端无法确认结果或 hold 终态，会返回
`reason == ArmMotionStatus.EXECUTION_STATE_UNKNOWN`、`execution_state_known=False`、
`executed=None`。这不表示“没有运动”；客户端会阻止新目标，此时先观察机器人并在可能
继续运动时按硬件急停，不得按普通可重试失败处理。

`move_to`、`move_by` 和 `move_joints` 是阻塞调用。需要在动作中途取消时，在另一个调用线程或
上层取消回调中调用同一客户端的 `cancel_result = arm.cancel()`。该方法本身会等待
server 的取消、hold 和 Action 终态，并返回同样结构的 `ArmResult`；没有活动目标时返回
`None`。原阻塞调用也会随后返回。不要用“立即发下一个目标”代替 cancel；下一个同臂
目标在旧目标活动或终态未知时会返回 `BUSY`。

`cancel(timeout_sec=...)` 的 timeout 从取得客户端内部控制锁后开始，不包含等待另一个
`cancel()`/`close()` 调用释放锁的时间。不要并发调用这些控制方法；Feedback callback
只做短小的状态转发，不要在 callback 内调用 `cancel()` 或 `close()`。如果 `close()`
报告 ROS worker 未退出，实体会被刻意保留以避免并发销毁；确认机器人状态后结束该进程，
不要继续复用该客户端。

## 7. ROS 2 Action 命令示例

本节首先用 goal `execute: false`。请在执行命令前从当前 FK、上层定位结果或已验证的
比赛点位中取得绝对目标。命令中用 `read` 而不是提供一个无法确认安全的通用数值。

### 7.1 末端绝对位置

```bash
read -r -p "target x y z in base_link (m): " TARGET_X_M TARGET_Y_M TARGET_Z_M

ros2 action send_goal \
  /duojin/arm/left/move_to \
  duojin_interfaces/action/MoveArmPose \
  "{target_pose: {header: {frame_id: base_link}, pose: {position: {x: ${TARGET_X_M}, y: ${TARGET_Y_M}, z: ${TARGET_Z_M}}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, keep_current_orientation: true, timeout_sec: 0.0, execute: false}" \
  --feedback
```

右臂只把端点中的 `left` 改为 `right`。本版 `keep_current_orientation` 不能改为
`false`。上述命令先以 `execute: false` 做 preview；在执行模式启动后，对已审核的同一目标
改为 `execute: true` 会真实发布 Pose 并等待到位。

### 7.2 末端相对位移

```bash
ros2 action send_goal \
  /duojin/arm/left/move_by \
  duojin_interfaces/action/MoveArmRelative \
  "{delta: {header: {frame_id: base_link}, vector: {x: 0.01, y: 0.0, z: 0.0}}, keep_current_orientation: true, timeout_sec: 0.0, execute: false}" \
  --feedback
```

上例只做“沿 `base_link` x 轴 +1 cm”的 preview。原生 ROS 调用不会自动填 frame。
在 `./start.sh --enable-arm-motion` 启动的环境中，将同一 Goal 改为 `execute: true` 会真实执行。

### 7.3 终端实时查看末端坐标

```bash
ros2 run duojin_robot_interface arm_pose_display --ros-args -p arm:=left
```

修改为 `-p arm:=right` 查看右臂。程序持续输出 frame、xyz（m）、四元数和 ROS
stamp；`Ctrl-C` 只退出这个只读显示程序，不会停止 API 或 SDK。也可直接查看原始消息：

```bash
ros2 topic echo /duojin/arm/left/current_pose
```

### 7.4 六关节绝对角

```bash
read -r -p "target q1 q2 q3 q4 q5 q6 (rad): " Q1 Q2 Q3 Q4 Q5 Q6

ros2 action send_goal \
  /duojin/arm/right/move_joints \
  duojin_interfaces/action/MoveArmJoints \
  "{positions_rad: [${Q1}, ${Q2}, ${Q3}, ${Q4}, ${Q5}, ${Q6}], speed_scale: 0.2, timeout_sec: 0.0, execute: false}" \
  --feedback
```

左臂只把端点中的 `right` 改为 `left`。不要传关节增量；六个值全部是绝对 rad。

ROS 2 Humble 的 `ros2 action send_goal` 命令不应被当作比赛程序的取消机制。比赛代码使用
`ArmClient.cancel()` 并等待结果；终端 `Ctrl-C` 或直接 kill server 不能代替已确认的
Action cancel。

## 8. 当前初始安全参数

标准 launch 的运行参数文件是
`src/duojin_robot_interface/config/arm_motion.yaml`，安装后由 launch 从包 share 目录读取。
服务端代码同时声明同类型的 fallback 默认值；修改参数时必须确保两处值保持一致，
标准运行一律使用本手册的 launch，不直接 `ros2 run` 绕过参数文件。
不要修改工控机 `install/` 中的生成副本；源码配置变更后按第 2 节重新构建。

截至 2026-07-29，除布尔执行门、frame 名和允许的节点名外，下表所有数值都只是
根据同版 SDK 证据选定的保守工程初值，**尚未在本机器人完成 L4 真机验证**。
这些值不是厂商保证，也不能自行证明轨迹、停止或碰撞安全。

| 参数 | 默认值 | 合法值/约束 | 用途 |
| --- | --- | --- | --- |
| `execute` | `false` | bool；只在清场后通过 launch 显式覆盖 | server 级真机执行门 |
| `ik_solver_frame` | `torso_link3` | 非空 frame 名 | Relaxed IK 实际数值基坐标系 |
| `public_pose_frame` | `base_link` | 非空 frame 名 | `current_pose` 对外统一坐标系 |
| `pose_publish_period_sec` | `0.05 s` | 有限且 `> 0` | 检查新 FK 样本并发布公开位姿的周期 |
| `feedback_freshness_sec` | `0.10 s` | 有限且 `> 0` | 关节/末端反馈最大 monotonic 接收间隔 |
| `feedback_wait_timeout_sec` | `3.0 s` | 有限且 `> 0` | 接单后等待反馈和 SDK 订阅端就绪 |
| `tf_timeout_sec` | `0.25 s` | 有限且 `> 0` | 单次 TF 转换等待上限 |
| `ik_response_timeout_sec` | `1.0 s` | 有限且 `> 0` | Pose 目标后等待新 Relaxed IK 关节目标 |
| `poll_period_sec` | `0.02 s` | 有限且 `> 0` | 反馈、取消和到位轮询周期 |
| `default_pose_timeout_sec` | `15.0 s` | 有限且 `> 0`，且 `<= maximum_timeout_sec` | Pose Goal 传 `0.0` 时的执行超时 |
| `maximum_timeout_sec` | `60.0 s` | 有限且 `> 0` | 客户端可请求的 Goal 超时上限 |
| `hold_timeout_sec` | `2.0 s` | 有限且 `> 0` | 取消/超时/异常后确认 current-position hold |
| `settle_duration_sec` | `0.25 s` | 有限且 `> 0` | 误差带内必须连续保持的时间 |
| `pose_position_tolerance_m` | `0.015 m` | 有限且 `> 0` | 末端位置到位阈值 |
| `pose_orientation_tolerance_rad` | `0.05 rad` | 有限且 `> 0` | 保持朝向的到位阈值 |
| `joint_position_tolerance_rad` | `0.02 rad` | 有限且 `> 0` | 六关节最大绝对误差阈值 |
| `controller_joint_lower_rad` | `[-2.8, 0.0, -3.14, -1.57, -1.57, -2.8]` | 6 个有限数，每轴 lower `<` upper | Joint Tracker 有效下限 |
| `controller_joint_upper_rad` | `[2.8, 3.14, 0.0, 1.57, 1.57, 2.8]` | 6 个有限数，每轴 upper `>` lower | Joint Tracker 有效上限 |
| `joint_limit_margin_rad` | `0.03 rad` | 有限且 `> 0`，留出的每轴区间必须非空 | 在控制器限位内再缩进 |
| `maximum_joint_delta_rad` | `0.35 rad` | 有限且 `> 0` | 单 Goal 每轴最大绝对变化 |
| `safe_joint_velocity_rad_s` | `[0.25, 0.25, 0.25, 0.25, 0.25, 0.25] rad/s` | 恰好 6 个有限正数 | 关节目标速度上限，再乘 Goal `speed_scale` |
| `hold_joint_velocity_rad_s` | `0.10 rad/s` | 有限且 `> 0` | best-effort current-position hold 的速度字段 |
| `maximum_cartesian_delta_m` | `0.08 m` | 有限且 `> 0` | 单 Pose Goal 与当前末端的最大直线距离 |
| `allowed_left_joint_publishers` | `[/relaxed_ik_left]` | string array | 左臂关节目标话题上唯一允许的外部发布者 |
| `allowed_right_joint_publishers` | `[/relaxed_ik_right]` | string array | 右臂关节目标话题上唯一允许的外部发布者 |

Joint Goal 传 `timeout_sec=0.0` 时不使用 `default_pose_timeout_sec`，而是按当前角、
目标角、`safe_joint_velocity_rad_s` 和 `speed_scale` 动态计算，最少 `5 s`、最多
`maximum_timeout_sec`。

| 项目 | 初值 |
| --- | --- |
| 关节命令速度上限 | 每轴 `0.25 rad/s × speed_scale`；Python 默认 scale `0.2` |
| 单次关节变化 | 每轴不超过 `0.35 rad` |
| 单次末端直线距离 | preview 和 execute 都不超过 `0.08 m`；首次 L4 只测 `0.01 m` |
| 关节到位 | 最大绝对误差 `≤ 0.02 rad`，连续 `≥ 0.25 s` |
| 末端到位 | 位置误差 `≤ 0.015 m`、朝向误差 `≤ 0.05 rad`，连续 `≥ 0.25 s` |
| 反馈新鲜度 | monotonic 接收间隔 `≤ 0.10 s` |

Pose 目标经 Relaxed IK 产生 joint position，该链路不提供可由本 API 证明的笛卡尔速度
上限。更关键的是，输出在被本项目观察到之前已经到达 Joint Tracker。执行模式允许这条
实验性路径：服务端在发布后检查新 IK 输出，再持续检查 FK 末端误差。这不是发布前关节安全门。

## 9. 状态、BUSY 和常见失败

`ArmMotionStatus.outcome` 先区分 `SUCCESS`、`RETRYABLE_FAILURE`、`FATAL_FAILURE`、
`CANCELLED` 和 `TIMEOUT`，`reason` 再给出更具体的命名常量。ROS CLI 会把它们显示为
整数；Python 代码应与 `ArmMotionStatus` 常量比较。

| outcome/reason 符号 | 含义 | 处理 |
| --- | --- | --- |
| reason `PREVIEW_COMPLETE` | 目标和当前状态已校验，未执行 | 确认 `outcome=SUCCESS`、`succeeded=True` 且 `executed=False`，核对 frame、绝对值和安全路径 |
| outcome `SUCCESS` + reason `NONE` + `executed=true` | 真实反馈在误差带内持续足够时间 | 仍检查最终误差，再进入下一步 |
| reason `EXECUTION_DISABLED` | goal `execute=true`，但 server 仍是 `execute=false` | 不会发布运动目标；预览时保持 goal `execute=false`，真机执行则重做安全检查后重启唯一 server |
| reason `POSE_EXECUTION_UNSAFE` | 仅为首个 preview-only 版本保留的线上兼容码 | 当前服务端不会主动返回；如收到则说明客户连到了旧 server，重新构建并只保留一个 API |
| reason `BUSY` | 同一手臂已有活动 goal | 不重试抢占；cancel 原 goal、等待返回后再发新目标 |
| reason `SERVER_UNAVAILABLE` | Python 客户端在时限内未找到 server/结果 | 查 server 进程、Action 端点、source 顺序和 `ROS_DOMAIN_ID`；不能把客户端超时当作机械臂已停止 |
| reason `INVALID_GOAL` / `OUT_OF_RANGE` | 长度、NaN/Inf、朝向模式、关节限位或单次变化不合法 | 修正上层目标；不要在客户端静默裁剪后继续运动 |
| reason `SDK_NOT_READY` / `FEEDBACK_STALE` | SDK 订阅端、反馈或时效不满足 | 停止执行，检查 CAN、HDAS、Joint Tracker 和实时反馈 |
| reason `TF_UNAVAILABLE` | 目标 frame 不能转到公共/求解 frame | 检查 `frame_id`、robot_state_publisher 和 `base_link` ↔ `torso_link3` TF；不要只改 frame 名 |
| reason `CONTROL_CONFLICT` | 遥操或其他发布者在争用目标 | 识别并停止冲突所有者；不要为绕过检查修改数量限制 |
| outcome `TIMEOUT` / `CANCELLED` | 未在时限内到位，或客户端取消 | server 会尝试一次最新关节位置 hold；确认实机已停止后才处理下一步 |
| reason `IK_NO_RESPONSE` | Pose 目标后没有可用的 IK/运动反馈 | 保持停止，查 Relaxed IK 日志、目标 frame 和可达性 |
| reason `STOP_FAILED` | 软件 hold 无法发送或无法确认 | 立即使用硬件急停，不再发目标 |
| reason `EXECUTION_STATE_UNKNOWN` | 客户端无法确认远端终态，`executed=None` | 不得重试或发新目标；观察机器人，存在继续运动可能时立即急停，保留日志后重启整套环境 |
| reason `INTERNAL_ERROR` | server 发生未预期异常 | 如仍有运动则急停；保留 server 日志和目标，不自动重试 |

## 10. 故障排查

### `Package 'duojin_robot_interface' not found` 或 Action 类型不存在

在工控机重新执行 `./scripts/build_robot.sh`，然后关闭旧终端或重新按 SDK → overlay
顺序 source。修改 `.action` 后仅 source 旧 `install/` 不会生成新类型。

### Action 端点不可用

确认 `duojin_arm_api` tmux 会话仍在运行，然后比较两个终端的
`echo ${ROS_DOMAIN_ID}`，再执行：

```bash
ros2 action list -t | grep '^/duojin/arm/'
ros2 node list
tmux capture-pane -pt duojin_arm_api
```

### 始终只返回 preview 或 `EXECUTION_DISABLED`

真机关节执行需确认环境由 `./start.sh --enable-arm-motion` 启动，且本 goal 也显式为
`execute=true`。预览时 goal 保持 `execute=false`；preview server 收到 goal
`execute=true` 时返回 `EXECUTION_DISABLED` 是正常安全门结果。
不要为了消除 preview 或该错误而同时启动第二个 server。

### `move_to`/`move_by` 返回 `POSE_EXECUTION_UNSAFE`

当前源码已不会主动返回该兼容码。如仍出现，检查工控机是否已切换到最新分支、重新
执行 `./scripts/build_robot.sh`，并确认图中只有一个 API server。

### 反馈过期或 SDK 未就绪

不要只看到话题名就认为硬件正常。只读检查左臂示例：

```bash
ros2 topic echo /hdas/feedback_arm_left --once
ros2 topic echo /relaxed_ik/motion_control/pose_ee_arm_left --once
ros2 topic info -v /motion_target/target_pose_arm_left
```

超时没有一帧实时消息时，检查供电、急停、CAN 和 SDK tmux 日志；不要绕过新鲜度门。

### TF 不可用或坐标明显不合理

```bash
ros2 run tf2_ros tf2_echo base_link torso_link3
ros2 topic echo /relaxed_ik/motion_control/pose_ee_arm_left --once
```

核对 pose 消息的 `header.frame_id`。厂商 Relaxed IK 直接使用数值但不使用输入 header 做 TF，
因此不能用修改 `frame_id` 字符串代替真实转换。

### 控制冲突或多份 server

```bash
tmux list-sessions
ros2 action info /duojin/arm/left/move_to
ros2 topic info -v /motion_target/target_pose_arm_left
ros2 topic info -v /motion_target/target_joint_state_arm_left
```

先识别每个节点/发布者的所有权，再停掉重复 API、Gello、VR、`r1lite_teleop` 或遗留
诊断节点。不得直接 kill 一个尚在运动的 Action server；先 cancel 并确认机械臂停止。

### 机械臂运动异常或软件无法停止

立即按硬件急停。人员安全和硬件停止确认优先于继续查日志。急停后不要自动恢复或
重发旧目标；按运维流程检查零点、反馈、中断原因和现场状态。

## 11. 停止顺序

正常停止按以下顺序：

1. 上层停止产生新目标。
2. 对活动 goal 调用 `ArmClient.cancel()`，等待最终结果并肉眼确认机械臂已停止。
3. 在另一工控机终端停止 API 和整个 SDK；脚本会先向 `duojin_arm_api` 发送 `Ctrl-C`，
   在反馈仍在线时等待 Action 收尾，再关闭厂商 SDK：

   ```bash
   cd ~/duojin_ws
   ./stop.sh
   ```

4. 确认 SDK/tmux 和当前用户的残留 ROS 2 进程已退出，再按机器人运维规程处理供电。

仅在 ready Shell 输入 `exit` 不会停止 tmux 中的 SDK。遇到无法确认的运动、取消失败或
`STOP_FAILED` 时，不再等待软件停止顺序，直接使用硬件急停。

## 12. L2–L4 验证流程

本地开发机最多只能验证到 L0/L1，本次实际通过项以交付说明为准。下列验证必须在
工控机/真机执行，在完成前不得
宣称机械臂 API 已真机验证。

### L2：工控机构建

1. 记录 `git log -1 --oneline` 和 SDK `package_info.txt` 版本。
2. 执行 `./scripts/build_robot.sh`，保留完整 `colcon` 结果。
3. 重新 source SDK 和 overlay，运行三个 `ros2 interface show` 命令。
4. 无参数运行 `./start.sh`，确认自动启动的六个 Action 端点与两个 pose 话题类型正确。

### L3：真实 ROS 图的只读/preview

1. 仍清场并握住急停，运行 `./start.sh`，不绕过任何 `[FAIL]`。
2. 使用 `start.sh` 自动启动的 preview server，记录每个端点只有一个 server。
3. 对左右臂各记录一帧 `/hdas/feedback_arm_*` 和 `/relaxed_ik/motion_control/pose_ee_arm_*`，
   核对 header frame、数组长度、数值和实际姿态。
4. 记录 `base_link` ↔ `torso_link3` TF，并查验两个 `current_pose` 的 frame、数值和更新频率；
   再用已知安全的当前/近邻绝对目标与小相对增量 preview 六个 Action。
5. 使用 `ArmClient` 时，六次结果都应为 `succeeded=True`、`executed=False`、
   `reason == ArmMotionStatus.PREVIEW_COMPLETE`；使用 ROS CLI 时核对 result 中的
   `status.outcome/status.reason/status.executed` 对应同一状态。同时确认
   API 没有发出真机运动目标。
6. 记录实测的反馈频率、QoS、frame 和发布者数量，将“documented”与“observed”证据区分。

### L4：单手臂低风险真机运动

1. 从模板建立记录：

   ```bash
   cd ~/duojin_ws
   cp docs/templates/robot-test-record.md \
     docs/test-records/YYYY-MM-DD-arm-motion-api.md
   ```

2. 一次只测一条手臂。底盘制动、工作区清空、操作员握住急停，其他所有机械臂
   目标发布者退出。
3. 先测 `move_joints`：使用实测当前值生成每轴变化 `≤ 0.15 rad` 的目标，
   `speed_scale=0.2`；先保持 goal `execute=false` 并人工审核。
4. 用 `./stop.sh` 完整停止 preview 环境，重新检查现场，再用唯一的
   `./start.sh --enable-arm-motion` 启动；将已审核 goal 改为 `execute=true`，单次执行。
5. 记录目标、实际反馈、最大误差、到位时间、是否超调、最终 result 和安全退出状态；
   同类小动作每臂至少 3 次。
6. 在仍可随时急停的小动作中分别验证一次 cancel 和超时，确认 result 正确且 hold
   后无继续原目标的运动。
7. 关节路径通过后，读取当前 `current_pose`，先对当前绝对坐标做 `move_to` preview，
   再对 `base_link` Z `+0.01 m` 做 `move_by` preview。审核后依次将同一 Goal 改为 `execute=true`，
   记录 Action 的 `executed`、最终 xyz、位置/朝向误差和到位时间。
8. 验证同臂第二个 goal 返回 `BUSY`，左右臂的互斥状态互不锁死。首次不要同时运动
   双臂；可用一臂的活动/取消状态加另一臂 preview 来验证独立性。
9. 任何参数调整一次只改一个主要变量，在记录中保留调整前后数据。

L4 完成后仍不代表有碰撞规划。把该 API 接入感知或比赛状态机前，还需要上层分别验证
目标时效、可达性、底盘制动、物体/货架碰撞余量和失败恢复。
