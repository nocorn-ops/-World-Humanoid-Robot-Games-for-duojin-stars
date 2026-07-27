# duojin_ws

夺锦之星的独立 ROS 2 overlay 工作区。仓库只保存赛队源码，不包含也不修改
Galaxea 厂商 SDK。

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

本地开发机没有 ROS 2 Humble，因此本地只改源码和做静态检查。构建与真机运行均在
工控机完成。不要提交或同步 `build/`、`install/`、`log/`。

首次使用、完整操作顺序及故障排查见 [`docs/项目使用说明.md`](docs/项目使用说明.md)。

## 标准部署流程

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

### 3. 启动完整 SDK

```bash
cd ~/duojin_ws
./scripts/start_robot_sdk.sh
```

脚本严格执行机器人要求的启动顺序：

```bash
bash ~/setup_can.sh
bash ~/can.sh
cd ~/galaxea/install_430/startup_config/share/startup_config/script
./robot_startup.sh boot ../sessions.d/ATCStandard/R1LITEBody.d/
```

等待 30 秒后，脚本会检查 `/motion_target/` 话题是否出现。项目不再自行启动或补启动
Joint Tracker、Relaxed IK、HDAS 等单个厂商节点，避免版本、参数和控制模式不一致。

### 4. 检查控制链并运行项目节点

```bash
cd ~/duojin_ws
./scripts/start_arm_environment.sh left
./scripts/run_arm_ik_test.sh
```

第一条命令只检查 SDK 节点、机械臂反馈和 IK 输出，不启动任何厂商节点。第二条命令
默认只预览目标；确认安全后才可执行：

```bash
./scripts/run_arm_ik_test.sh --ros-args -p execute:=true
```

`R1LITEBody.d` 同时会启动 `r1lite_teleop`。自主控制前必须停止该 tmux session，避免
Gello 遥操作与赛队程序同时发布机械臂目标。执行模式检测到该 session 时会拒绝运动。

## 工控机更新

```bash
cd ~/duojin_ws
./scripts/update_robot.sh
```

该脚本在工控机存在未提交修改时拒绝更新，并只执行 fast-forward 拉取，随后重新构建。

## 目录

```text
duojin_ws/
├── docs/
│   ├── 项目使用说明.md             # 从开发到真机运行的完整操作手册
│   └── r1_lite_interfaces.md       # 官方文档与联调记录校验后的二开接口
├── src/
│   └── duojin_arm_test/            # R1 Lite 末端位姿→IK→关节跟踪验证
├── scripts/
│   ├── build_robot.sh              # 在工控机构建 overlay
│   ├── start_robot_sdk.sh          # CAN + 完整 install_430 启动
│   ├── start_arm_environment.sh    # 只检查机械臂 SDK 控制链
│   ├── run_arm_ik_test.sh          # 运行赛队测试节点
│   ├── update_robot.sh             # 安全拉取并重新构建
│   └── deploy_to_robot.sh          # 无 Git 远端时的 rsync 备用方式
└── .gitignore
```

机械臂真机流程见 [`src/duojin_arm_test/README.md`](src/duojin_arm_test/README.md)，
底盘和躯干话题见 [`docs/r1_lite_interfaces.md`](docs/r1_lite_interfaces.md)。
