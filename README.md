# duojin_ws

夺锦之星的独立 ROS 2 overlay 工作区。仓库只保存赛队源码，不包含也不修改
Galaxea 厂商 SDK。

## 开发守则

所有人工开发和 AI 编程代理在修改代码前，都必须完整阅读并遵守
[`AGENTS.md`](AGENTS.md)。新增跨模块或运动功能时，先从
[`docs/templates/feature-spec.md`](docs/templates/feature-spec.md) 建立可验收的任务规格；
单子系统及以上真机测试使用
[`docs/templates/robot-test-record.md`](docs/templates/robot-test-record.md) 留存环境、步骤、
数据和安全退出状态。

## 固定架构

机器人工控机上的环境固定为：

```text
~/galaxea/install_430       厂商 underlay，负责 HDAS、Mobiman、IK 等底层节点
~/duojin_ws                 赛队 overlay，只负责自己的节点和比赛逻辑
```

`source` 只把包、消息和动态库加入当前 shell；真正通信的是两个工作区启动出的
ROS 2 节点。项目 shell 必须先加载 SDK，再加载本工作区：

```bash
source ~/galaxea/install_430/setup.bash
source ~/duojin_ws/install/setup.bash
```

以上 `~` 均指 **R1 Lite 工控机用户 `r1lite` 的主目录 `/home/r1lite`**，不是本地
开发机的 `/home/vedal`。因此工控机默认实际路径是：

```text
/home/r1lite/galaxea/install_430
/home/r1lite/duojin_ws
```

项目脚本会根据脚本自身位置寻找 `duojin_ws`，不依赖执行命令时所在的目录；SDK
路径则固定从工控机的 `${HOME}/galaxea/install_430` 取得。比赛源码和配置中不得写入
本地开发机的绝对路径。

本地开发机没有 ROS 2 Humble，因此本地只改源码和做静态检查。构建与真机运行均在
工控机完成。不要提交或同步 `build/`、`install/`、`log/`。

首次使用、完整操作顺序及故障排查见 [`docs/项目使用说明.md`](docs/项目使用说明.md)。

## 一键启动和关闭

项目已经在工控机编译完成后，日常启动只需要：

```bash
cd ~/duojin_ws
./start.sh
```

`start.sh` 自动配置 CAN、启动完整 `install_430` SDK、等待 ROS 话题就绪、关闭
`r1lite_teleop`，并检查底盘、躯干、双臂、双夹爪、IMU 与三组相机的完整控制/反馈链。
检查通过后，它还会在 `duojin_arm_api` tmux 会话中自动启动唯一的机械臂 API
preview server，最后进入带 `[duojin]` 提示符的环境 Shell。此时比赛程序可以直接调用
六个 Action、两个实时末端坐标话题或 Python API；默认请求只做预览，不发布运动目标。

```bash
# 仅在清场、底盘制动、无外部控制者且操作员握住急停后使用
./start.sh --enable-arm-motion
```

该显式机械臂验证模式仍检查所有运动控制、反馈、IK 和末端位姿链路；
相机仍会被检查并打印，但缺帧只记为 `[WARN]`，不阻止手动机械臂 API 验证。
无参数的整机 preview 模式仍将相机缺帧记为 `[FAIL]`。

无参数 `./start.sh` 永远不授权真机运动。真机关节或末端运动需要以上正向启动参数，且每个
Goal 仍必须再显式设置 `execute=true`；Python 函数的默认 `execute=False` 始终只 preview。
`move_to`/`move_by` 在这两道门同时打开后会真实进入厂商 Relaxed IK，
服务端会继续检查 IK 输出和末端反馈并等待到位。厂商会在项目事后校验前
将 IK 关节目标发给 Joint Tracker，因此该路径属于显式启用的实验性执行。
左右臂分别暴露
`/duojin/arm/{left,right}/{move_to,move_by,move_joints}`
Action，比赛 Python 程序推荐使用
`duojin_robot_interface.arm_client.ArmClient`。完整前置条件、ROS 命令、Python
示例和分级真机验收见
[`docs/runbooks/arm-motion-api.md`](docs/runbooks/arm-motion-api.md)。

在 `start.sh` 打开的 `[duojin]` Shell 中，最小 Python 调用是：

```python
from duojin_robot_interface import get_pose, move_by, move_to

def preview_left_target(target_x_m, target_y_m, target_z_m):
    return move_to(
        target_x_m, target_y_m, target_z_m,
        arm="left", frame_id="base_link", execute=False,
    )
```

三个坐标是 `base_link` 中的绝对米值，不是相对增量；`execute=False` 是安全 preview。

相对位移和当前坐标查询的最小用法：

```python
pose = get_pose(arm="left")
preview = move_by(0.01, 0.0, 0.0, arm="left", execute=False)
print(pose.frame_id, pose.position_xyz, pose.orientation_xyzw)
```

`move_by` 默认在 `base_link` 轴方向上加位移，单位为 m，并保持当前朝向。终端实时显示：

```bash
ros2 run duojin_robot_interface arm_pose_display --ros-args -p arm:=left
```

现有诊断脚本仍可用于单次接口检查，但只允许 preview：

```bash
./scripts/run_arm_ik_test.sh
```

也可以直接执行自己的 `ros2 launch ...`。输入 `exit` 只退出环境 Shell，不关闭已经在
tmux 中运行的 SDK；完整关闭请从另一个终端执行 `./stop.sh`。

停止完整 SDK 和当前用户残留的 ROS 2 进程：

```bash
cd ~/duojin_ws
./stop.sh
```

`start.sh` 和 `stop.sh` 都不执行 Git 更新、项目编译或比赛 launch；`stop.sh` 会先在
SDK 反馈仍在线时停止机械臂 API，再关闭厂商 SDK。全机只能运行一份 API server。

## 部署流程

### 1. 本地开发并上传

```bash
git add <本次修改的文件>
git commit -m "..."
git push
```

### 2. 工控机拉取并构建 overlay

```bash
cd ~/duojin_ws
git pull --ff-only
./scripts/build_robot.sh
```

`build_robot.sh` 只加载固定的 `~/galaxea/install_430/setup.bash`，然后执行
`colcon build --symlink-install`。SDK 的生成式 setup 文件会继续加载其构建时使用的
`/opt/ros/humble`；若工控机缺少 Humble，脚本会明确失败。

### 3. 启动环境

```bash
cd ~/duojin_ws
./start.sh
```

脚本严格执行机器人要求的启动顺序：

```bash
bash ~/setup_can.sh
bash ~/can.sh
cd ~/galaxea/install_430/startup_config/share/startup_config/script
./robot_startup.sh boot ../sessions.d/ATCStandard/R1LITEBody.d/
```

等待 30 秒后，脚本会检查 `/motion_target/` 话题，自动关闭遥操作，并检查全部设备链路。
项目不再自行补启动 Joint Tracker、Relaxed IK、HDAS 等单个厂商节点。

### 4. 预览旧 IK 诊断，或运行统一 API

```bash
cd ~/duojin_ws
./scripts/run_arm_ik_test.sh
```

该旧脚本只用于 preview，禁止传入 `execute:=true`。厂商 Relaxed IK
在收到 Pose 后会直接向 Joint Tracker 发布关节目标，这个副作用发生在
项目能完成事前关节限幅之前；因此旧脚本仍保持 preview-only。
需要真实末端运动时应使用统一 API 的
`move_to(..., execute=True)` 或 `move_by(..., execute=True)`，不再让诊断节点成为第二个控制者。

需要真机运动时，使用统一 API 的 `move_to`、`move_by` 或 `move_joints`：以
`./start.sh --enable-arm-motion` 显式打开 server 级许可，完成运行手册的
前置检查后，再对经验证的目标显式传入 `execute=true`。具体步骤见
[`docs/runbooks/arm-motion-api.md`](docs/runbooks/arm-motion-api.md)。

## 工控机更新

```bash
cd ~/duojin_ws
./scripts/update_robot.sh
```

该脚本在工控机存在未提交修改时拒绝更新，并只执行 fast-forward 拉取，随后重新构建。

## 目录

```text
duojin_ws/
├── start.sh                       # CAN + SDK + 关闭遥操作 + 整机链路检查
├── stop.sh                        # 停止 SDK 和当前用户的 ROS 2 进程
├── docs/
│   ├── 项目使用说明.md             # 从开发到真机运行的完整操作手册
│   ├── r1_lite_interfaces.md       # 官方文档与联调记录校验后的二开接口
│   └── runbooks/arm-motion-api.md  # 机械臂 API 使用、安全和真机验证
├── src/
│   ├── duojin_interfaces/          # 赛队自定义机械臂 Action 契约
│   ├── duojin_robot_interface/     # SDK 适配、安全门与 Python 客户端
│   └── duojin_arm_test/            # R1 Lite 末端位姿→IK→关节跟踪验证
├── scripts/
│   ├── build_robot.sh              # 在工控机构建 overlay
│   ├── start_robot_sdk.sh          # CAN + 完整 install_430 启动
│   ├── check_robot_control_chains.sh # 只读检查整机设备链路
│   ├── start_arm_environment.sh    # 只检查机械臂 SDK 控制链
│   ├── run_arm_ik_test.sh          # 旧笛卡尔 IK 链路的 preview-only 诊断
│   ├── update_robot.sh             # 安全拉取并重新构建
│   └── deploy_to_robot.sh          # 无 Git 远端时的 rsync 备用方式
└── .gitignore
```

旧 IK 链路的预览诊断见
[`src/duojin_arm_test/README.md`](src/duojin_arm_test/README.md)；统一机械臂 API 的安全
真机流程见 [`docs/runbooks/arm-motion-api.md`](docs/runbooks/arm-motion-api.md)。底盘和躯干话题见
[`docs/r1_lite_interfaces.md`](docs/r1_lite_interfaces.md)。
