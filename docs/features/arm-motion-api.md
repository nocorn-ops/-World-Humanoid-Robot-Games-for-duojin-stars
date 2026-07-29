# 功能规格：arm-motion-api

## 0. 元数据

- 状态：`implementing`
- 风险等级：`motion-high`
- 创建/更新日期：2026-07-29
- 负责人：Duojin Team
- 目标 Git 分支/提交：当前工作分支
- 关联比赛阶段：`通用能力`
- 最终运行位置：纯逻辑在本地与工控机运行，ROS 节点只在 R1 Lite 工控机运行

## 1. 最终结果

> 完整 SDK 已启动后，比赛程序能够分别通过 ROS 2 Action 或 Python `ArmClient` 预览
> 左/右臂末端绝对或相对三维位置，并读取持续发布的当前末端位姿；在操作员明确允许执行时请求六关节绝对角度；关节服务端
> 在反馈连续进入配置误差带后才返回动作成功，并在忙、取消、超时、反馈过期、TF
> 不可用、越界或控制冲突时返回结构化失败。末端物理执行在 IK 隔离前明确拒绝。

## 2. 范围

### 本次包含

- 左右臂各自独立的绝对/相对末端 preview Action 和关节位置 Action；保留末端执行字段但由
  强制安全门拒绝物理执行。
- `move_to(x, y, z)`、`move_by(dx, dy, dz)` 和 `get_pose()` Python 封装，保持目标校验时读取的最新末端朝向。
- 左右臂各一个默认 `base_link` 的新鲜末端 `PoseStamped` 公开话题和终端显示程序。
- 对外接受带 `frame_id` 的绝对坐标，默认使用 `base_link`，适配层转换到 IK 的实际
  `torso_link3` 数值坐标系。
- 默认 preview、显式执行许可、每臂 BUSY 互斥、限位、反馈新鲜度、到位、超时、取消
  和控制发布者冲突检查。
- 关节物理目标在取消、超时和进程退出时，向关节目标接口发送最新关节位置作为
  best-effort hold。
- 使用前提、启动、ROS 命令、Python 调用、错误码和逐级真机验证文档。

### 明确不包含

- 双臂同步轨迹、轨迹时间参数化、MoveIt、碰撞规划、自碰撞检测和奇异点保证。
- 夹爪、躯干、底盘控制或比赛任务状态机。
- 末端朝向修改；本版只接受 `keep_current_orientation=true`。
- 通过软件 Action 取代硬件急停，或承诺 hold 命令等同于硬件急停。
- 自动标定机械零点、修改 HDAS bias、自动 homing 或证明冷启动绝对零点正确。

## 3. 输入、输出与前置条件

| 类别 | 名称 | 类型 | 单位/坐标系/频率 | 来源或去向 | 有效条件 |
| --- | --- | --- | --- | --- | --- |
| 输入 | 末端目标 | `MoveArmPose.Goal` | m；Python 默认 `base_link`，原生 ROS Goal 必须显式给非空 `frame_id`；v1 用最新 TF 而不使用 Goal stamp；单次 | ROS/Python 客户端 | xyz 有限、TF 可用、距当前位姿不超过配置上限、保持当前朝向 |
| 输入 | 末端相对增量 | `MoveArmRelative.Goal` | m；`delta.header.frame_id` 定义轴；Python 默认 `base_link`；单次 | ROS/Python 客户端 | delta 有限、frame/TF 可用、增量不超过配置上限、保持当前朝向 |
| 输入 | 关节目标 | `MoveArmJoints.Goal` | rad；joint1..joint6；单次 | ROS/Python 客户端 | 恰好 6 项、有限、在控制器有效限位与 margin 内、单次变化不过限 |
| 输入 | 关节反馈 | `sensor_msgs/msg/JointState` | rad；诊断配置期望约 200 Hz，实际频率待 L3 观测 | `/hdas/feedback_arm_{left,right}` | 至少 6 个有限 position，以 monotonic 接收时刻判断新鲜度 |
| 输入 | 末端反馈 | `geometry_msgs/msg/PoseStamped` | m/rad；消息声明的 frame | `/motion_control/pose_ee_arm_{left,right}` | frame 非空、位姿有限、四元数有效、反馈新鲜 |
| 输出 | 末端 SDK 目标（预留） | `geometry_msgs/msg/PoseStamped` | 数值应在 `torso_link3`；单次 | `/motion_target/target_pose_arm_{left,right}` | 当前版本不发布；完成 IK I/O 隔离并重新验证后才可启用 |
| 输出 | 关节 SDK 目标 | `sensor_msgs/msg/JointState` | rad 与 rad/s；单次 | `/motion_target/target_joint_state_arm_{left,right}` | position/velocity 均严格 6 项；仅 execute 或 hold 时发布 |
| 输出 | 动作结果/反馈 | 三个自定义 Action | 结构化状态、误差、当前/最终值 | ROS/Python 客户端 | 只有真实反馈到位才是已执行 SUCCESS |
| 输出 | 当前末端位姿 | `PoseStamped` | 默认 `base_link`；新 FK 样本驱动 | ROS/Python/终端 | 不重发旧样本，读取超时明确失败 |

前置条件：

- 工控机已按 SDK underlay → 项目 overlay 编译，并通过 `./start.sh` 的完整链路检查。
- SDK 只运行一份；Joint Tracker、左右 Relaxed IK、全身 FK 和
  `robot_state_publisher` 已由厂商启动，项目不重复启动。
- `r1lite_teleop`、Gello、VR 或其他目标发布者不再争用控制；API 服务只运行一份。
- 底盘静止并制动，机械臂工作区清空，操作员握住急停；软件检查不替代这些前提。
- 真机关节执行必须用 `./start.sh --enable-arm-motion` 显式授权；preview 不发布运动目标。
- 末端物理执行在 IK 输入/输出隔离完成前强制返回 `POSE_EXECUTION_UNSAFE`。

## 4. 验收指标

| 指标 | 目标值 | 测量方法 | 所需重复次数 | 失败阈值 |
| --- | --- | --- | --- | --- |
| Python/ROS 契约 | 六个 Action、两个 pose 话题均可用 | `ros2 action list -t`、`ros2 topic list -t` 与 Python 示例 | L3 一次 | 任一端点或类型缺失 |
| preview 安全 | 0 条运动目标由 API 发布 | 无参数 `start.sh` 后调用并检查结果/话题 | 每个 Action 1 次 | 发布了目标或返回 executed=true |
| 关节闭环 | 最大绝对误差 ≤ 0.02 rad，持续 ≥ 0.25 s | 反馈与 Action result | 每臂 3 次小动作 | 超时或超误差 |
| 末端执行安全门 | `execute=true` 返回 `POSE_EXECUTION_UNSAFE` 且 0 条 Pose 目标 | Action result 与目标话题 | L1/L3 每臂 1 次 | 目标被发布或声称已执行 |
| 反馈安全门 | > 0.10 s 未收到反馈即拒绝/中止 | 停反馈或注入纯逻辑时钟 | L1、L3 各 1 次 | 继续发新目标或成功 |
| 并发互斥 | 同一臂第二目标返回 BUSY；左右臂互不锁死 | 并发 Action 调用 | L1/L3 各 1 次 | 同臂双目标同时执行 |
| 关节取消/超时 | 返回 CANCELLED/TIMEOUT 且发 hold | 低速关节小动作中取消/缩短超时 | 每臂各 1 次 L4 | 仍继续原目标或无明确 STOP_FAILED |

表中运动阈值是 2026-07-29 的保守工程初值，不是厂商保证；L4 后按实测更新。

## 5. 能力拆解

| ID | 单一职责 | 输入 | 输出 | 前置条件 | 成功判据 | 失败结果 | 验证层级 | 状态 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| CAP-ARM-01 | 校验关节目标 | 6 角度、speed scale、当前角度 | 有效速度与目标 | 新鲜关节反馈 | 全部有限且限位/单步检查通过 | INVALID_GOAL/OUT_OF_RANGE | L0/L1 | implementing |
| CAP-ARM-02 | 规范化末端目标 | 带 frame 的 xyz、当前末端 pose | solver-frame Pose | 新鲜 pose 与 TF | 保持当前朝向且完成两次 TF | TF_UNAVAILABLE/OUT_OF_RANGE | L1/L3 | implementing |
| CAP-ARM-03 | 仲裁单臂控制权 | arm 与 goal | 独占 token | server 唯一 | 同臂串行、异臂独立 | BUSY | L1/L3 | implementing |
| CAP-ARM-04 | 发布关节 SDK 目标 | 已校验 JointState | `/motion_target/target_joint_state_arm_*` 单次消息 | execute、订阅者、无冲突 | 发布一次且数组/QoS 符合证据；Pose 发布保持禁用 | SDK_NOT_READY/CONTROL_CONFLICT/OUT_OF_RANGE/POSE_EXECUTION_UNSAFE | L3/L4 | implementing |
| CAP-ARM-05 | 判断关节动作到位 | 目标与实时反馈 | 结果/Action feedback | 反馈新鲜 | 连续 0.25 s 在误差带；末端闭环待隔离后验证 | FEEDBACK_STALE/TIMEOUT | L1/L4 | implementing |
| CAP-ARM-06 | 取消并 hold | 取消/超时/退出与当前关节 | hold 目标和结果 | 新鲜关节反馈、订阅者存在 | hold 已发送且返回明确状态 | STOP_FAILED | L3/L4 | implementing |
| CAP-ARM-07 | 封装 Python 调用 | arm、xyz 或 6 角度 | `ArmResult` | Action server 在线 | 阻塞返回且可显式 cancel | SERVER_UNAVAILABLE/结构化状态 | L1/L3 | implementing |
| CAP-ARM-08 | 发布/查询末端位姿 | 新鲜 FK pose 和 TF | `current_pose`/`ArmPoseReading` | 反馈与 TF 有效 | 只发布新鲜已转换样本 | FEEDBACK_STALE/TF 错误/超时 | L1/L3 | implementing |

依赖顺序：

```text
CAP-ARM-01/02 → CAP-ARM-03 → CAP-ARM-04 → CAP-ARM-05/06 → CAP-ARM-07
```

## 6. SDK / ROS 接口证据

详细证据和未确认项见 `docs/interfaces/arm.md`。

| 接口 | 方向与类型 | 字段/单位/范围 | Frame/QoS/频率 | 前置节点 | 控制所有者 | 证据状态 |
| --- | --- | --- | --- | --- | --- | --- |
| `/motion_target/target_pose_arm_*` | 项目→SDK，`PoseStamped` | xyz m、quaternion | subscriber 实际忽略 header；BEST_EFFORT/VOLATILE/depth1 兼容 | Relaxed IK | API；不可有外部 pose publisher | documented |
| `/motion_target/target_joint_state_arm_*` | 项目/IK→SDK，`JointState` | 6 position rad + 6 正 velocity rad/s | tracker BEST_EFFORT/VOLATILE/depth10 | Joint Tracker | API 或 Relaxed IK；不得有 teleop | documented |
| `/hdas/feedback_arm_*` | SDK→项目，`JointState` | 使用 position[0:6] | BEST_EFFORT/VOLATILE/depth1；诊断期望 200 Hz | HDAS | HDAS | documented |
| `/motion_control/pose_ee_arm_*` | SDK→项目，`PoseStamped` | 全身 FK 的 gripper pose | frame/QoS/频率待 L3 实测 | eepose publisher | SDK | documented，frame 待 observed |
| TF `base_link ↔ torso_link3` | SDK→项目，TF | 刚体变换 | 最新有效 TF | robot_state_publisher | SDK | documented，时效待 observed |

尚待真机确认的假设：

- `/motion_control/pose_ee_arm_*` 的实际 `header.frame_id`、频率和 QoS 与全身 URDF一致。
- `JointState.velocity` 反馈是否稳定存在；本版到位只强依赖已确认的 position。
- 完成隔离后，单次 pose 目标能否在所有初始姿态可靠产生可先校验的 IK joint target。
- hold-current 对当前 Joint Tracker 版本的减速/停止效果；未经 L4 不称为硬停止。
- 机器人 `/opt/galaxea/body/hardware.json` 的 joint bias 与断电重启重复性。

## 7. 状态机与恢复

| 当前状态 | 成功条件 | 超时 | 可重试失败 | 致命失败 | 下一状态/安全动作 |
| --- | --- | --- | --- | --- | --- |
| IDLE | 获得单臂 token | 无 | BUSY | 无 | VALIDATING 或返回 BUSY |
| VALIDATING | 目标、反馈、TF、订阅者、发布者均通过 | readiness timeout | SDK_NOT_READY、FEEDBACK_STALE、TF_UNAVAILABLE、CONTROL_CONFLICT | INVALID_GOAL、OUT_OF_RANGE | PREVIEW/COMMANDING/返回失败 |
| PREVIEW | 完成全部只读校验 | 无 | 无 | 无 | 返回 PREVIEW_COMPLETE，executed=false |
| COMMANDING | 当前仅关节 SDK 目标单次发布；Pose 在此前失败 | goal timeout | 无 | publish/internal error | HOLDING 或 WAITING |
| WAITING | 连续稳定样本达标 | goal timeout | FEEDBACK_STALE | 无 | SUCCESS 或 HOLDING |
| HOLDING | 最新关节位置 hold 已发布 | hold timeout | 无 | STOP_FAILED | CANCELLED/TIMEOUT 或致命失败 |

重试上限与变化策略：

- 服务端不自动重试或自动改目标；上层收到 BUSY/STALE/TF/SDK_NOT_READY 后可在消除原因后重试。
- 同一臂新目标不抢占旧目标；必须先显式 cancel 并收到结果，再发送下一目标。
- IK 无响应、超时和反馈过期均停止继续发送原目标并尝试一次 hold，不无限重发。

## 8. 安全分析

- 涉及的执行器：左/右六自由度机械臂。
- preview 输出：完整校验、目标/误差日志和 PREVIEW_COMPLETE；不发布 `/motion_target/*`。
- 执行许可方式：server `execute:=false` 默认；只有 `start.sh --enable-arm-motion` 且每个
  goal 再显式设置 `execute=true`，关节目标才可能执行。goal `execute=false` 时 preview；
  goal `execute=true` 但 server 仍为 false 时返回 `EXECUTION_DISABLED`。Pose 即使两道门
  都打开也返回 `POSE_EXECUTION_UNSAFE`，直到 IK 输入/输出隔离完成。
- 位置/速度/频率/力限制：按 tracker 有效限位再留 0.03 rad；单关节变化默认 ≤ 0.35
  rad；末端单目标位移默认 ≤ 0.08 m；关节速度初值 0.25 rad/s×speed scale；单次发布，
  不修改力矩。所有值待 L4 标定。
- 反馈新鲜度与动作超时：monotonic 接收间隔默认 ≤ 0.10 s；关节默认动态超时、末端
  默认 15 s、总上限 60 s；误差带连续 ≥ 0.25 s 才到位。
- 控制冲突检查：每次执行前检查 pose 与 joint 目标 publisher；只允许本 API，joint 话题
  额外允许对应 Relaxed IK。文件锁阻止同一工控机重复 server。
- 取消、异常和进程退出时的停止动作：只尝试发布最新新鲜关节位置的低速 hold。厂商无
  已确认 stop Action/service，停止保证必须经 L4 验证；hold 失败时返回 STOP_FAILED 并要求急停。
- 人工清场和急停要求：执行前底盘静止/制动、单臂小范围、工作区无人无障碍、操作员
  始终握住急停。API 不做碰撞规划，合法数值不代表路径安全。

## 9. 文件与所有权计划

| 文件 | 所属包/目录 | 职责 | 新增或修改 | 不放在其他位置的原因 |
| --- | --- | --- | --- | --- |
| `src/duojin_interfaces/msg/ArmMotionStatus.msg` | duojin_interfaces | 稳定结果码 | 新增 | 跨 ROS 包契约只属于 interfaces |
| `src/duojin_interfaces/action/*.action` | duojin_interfaces | 三种 Action 契约 | 新增 | 标准消息不能表达反馈/取消/结构化结果 |
| `src/duojin_robot_interface/duojin_robot_interface/arm_domain.py` | robot_interface | 无 ROS 校验/误差/到位逻辑 | 新增 | SDK 适配领域拥有，且本地可测 |
| `src/duojin_robot_interface/duojin_robot_interface/arm_sdk_adapter.py` | robot_interface | 厂商 topic/QoS/TF/反馈边界 | 新增 | SDK 知识必须唯一收口 |
| `src/duojin_robot_interface/duojin_robot_interface/arm_execution.py` | robot_interface | Action 执行/恢复闭环 | 新增 | 属于安全适配，不放 manipulation |
| `src/duojin_robot_interface/duojin_robot_interface/arm_motion_server.py` | robot_interface | ROS 参数与 Action 装配 | 新增 | 单一运行职责 |
| `src/duojin_robot_interface/duojin_robot_interface/arm_client.py` | robot_interface | Python 简单接口 | 新增 | 公开契约的薄客户端 |
| `src/duojin_robot_interface/config/arm_motion.yaml` | robot_interface | 初始安全参数 | 新增 | 参数语义由适配包拥有 |
| `src/duojin_robot_interface/launch/arm_motion.launch.py` | robot_interface | 单能力启动 | 新增 | 该能力有独立启动价值，不需 bringup 反向依赖 |
| `src/duojin_robot_interface/test/*` | robot_interface | L1 纯逻辑测试 | 新增 | 与实现同包 |
| `docs/interfaces/arm.md` | docs/interfaces | SDK 证据 | 新增 | 统一接口事实来源 |
| `docs/runbooks/arm-motion-api.md` | docs/runbooks | 操作员使用手册 | 新增 | 运行流程不埋在代码 README |

新增依赖及机器人端安装方式：

- 使用 ROS 2 Humble 已有的 `rclpy`、`geometry_msgs`、`sensor_msgs`、`tf2_ros`、
  `tf2_geometry_msgs`、`rosidl_default_generators` 和 launch；不引入 pip/网络依赖。
- 在工控机加载 `install_430` 后运行项目现有 `./scripts/build_robot.sh` 生成接口和 Python 包。

## 10. 验证矩阵

| 层级 | 用例 | 命令/步骤 | 期望结果 | 实际证据 | 状态 |
| --- | --- | --- | --- | --- | --- |
| L0 | Python/Shell/XML/接口静态检查 | compileall、bash -n、XML parse、路径/规模、diff check | 全部通过 | 2026-07-29：全部通过；所有生产模块 ≤400 行、函数 ≤60 行 | passed |
| L1 | 校验、相对几何、误差、稳定窗口、互斥、生命周期、客户端映射/并发 | `python3 -m pytest -q src/duojin_robot_interface/test` | 正常与失败分支通过 | 2026-07-29：`96 passed in 0.13s` | passed |
| L2 | 工控机 ROS 构建 | `./scripts/build_robot.sh` | rosidl 与两个包构建成功 | 需工控机 | todo |
| L3 | 真实图 preview | 无参数 `start.sh` 自动启动 preview，调用六个 Action 并读两个 pose 话题 | 无运动消息，frame/反馈/QoS 记录完成 | 需工控机 | todo |
| L4 | 单臂低速关节小动作/取消 | 测试记录模板；每轴变化 ≤0.15 rad | 到位、超时、取消与 hold 数据符合指标；不测 Pose 物理执行 | 需机器人 | todo |
| L5 | manipulation 调用 | 上层用 Python/Action 组合能力 | 错误传播与恢复正确 | 不属本次真机交付 | todo |
| L6 | 比赛回归 | 多轮比赛流程 | 无控制权/接口回归 | 不属本次真机交付 | todo |

## 11. 决策与变更记录

| 日期 | 决策/变更 | 证据与理由 | 影响 |
| --- | --- | --- | --- |
| 2026-07-29 | 末端 v1 只保持当前朝向 | 用户明确选择 | Action 保留兼容字段，但 false 目标会被拒绝 |
| 2026-07-29 | 左右臂独立、同臂不抢占 | 用户明确选择，Action 可返回 BUSY/cancel | 六端点、每臂一个 arbiter |
| 2026-07-29 | public frame 默认 base_link，发布前转 torso_link3 | Relaxed IK 忽略 header 且 solver base 是 torso_link3 | TF 成为强制安全门 |
| 2026-07-29 | `start.sh` 自动启动 preview API | 用户期望一次启动后程序可直接调用；preview 不发布目标 | `--enable-arm-motion` 与 Goal execute 构成关节执行双门 |
| 2026-07-29 | 相对位移默认沿 `base_link` 轴，可显式指定其他 frame | 用户确认可支持并接受建议 | 服务端在目标校验时从新鲜当前位姿计算绝对目标 |
| 2026-07-29 | 公开位姿默认为 `base_link`、只发布新鲜 FK 样本 | 比赛程序需统一坐标与实时查询 | `start.sh` 就绪后话题和终端显示可直接使用 |
| 2026-07-29 | 暂停 `move_to`/`move_by` 物理执行 | Relaxed IK 在项目事前校验前直接发布 joint target，且可能晚于 hold | preview 可用；物理请求返回 POSE_EXECUTION_UNSAFE，等待 IK I/O 隔离 |
| 2026-07-29 | 使用 tracker 有效限位而非更宽 URDF 限位 | SDK 二进制接口证据 | 越界拒绝，不静默裁剪 |

## 12. 完成检查

- [ ] 验收指标有数据证据
- [ ] 主要失败与恢复分支已验证
- [ ] 运动功能具备 dry-run、限幅、超时和安全停止
- [x] 接口文档与运行手册已更新
- [ ] 真机记录已链接
- [x] 未执行的验证层级和剩余风险已明确
