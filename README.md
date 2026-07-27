# duojin_ws

夺锦之星的独立 ROS 2 overlay 工作区。这个目录只放赛队自己的代码，不包含、不修改
Galaxea 厂商 SDK。

## Git 部署模型

```text
开发机 duojin_ws
       │ git push
       ▼
GitHub / Gitee / 自建 Git 服务
       │ git clone / git pull --ff-only
       ▼
工控机 /home/r1lite/duojin_ws
                         │
                         ├── 依赖 /opt/ros/humble
                         └── 依赖 /home/r1lite/galaxea/install
```

工控机实际运行的是 `/home/r1lite/galaxea/install`。本地
`galaxea/install_430` 仅作为同版本 SDK 的只读接口快照，不应写入启动脚本或
代码的绝对运行时路径。

## 一键启动机械臂环境

首次构建后，在工控机执行：

```bash
cd /home/r1lite/duojin_ws
./scripts/start_arm_environment.sh left
```

脚本会加载 ROS 2、Galaxea SDK 和本工作区 overlay，复用已经存在的节点，
并只启动缺失的 Joint Tracker 与左臂 Relaxed IK。它本身不发布运动
目标。将 `left` 改为 `right` 可启动右臂环境。保持该终端运行；
按 `Ctrl+C` 只会关闭由该脚本启动的节点。

## 首次部署到工控机

先在 GitHub、Gitee 或自建 Git 服务创建空仓库，然后在开发机为本项目添加远程并
推送：

```bash
cd /home/vedal/WorkStation/世界人型机器人运动会/duojin_ws
git remote add origin <GIT_REPOSITORY_URL>
git push -u origin main
```

在工控机首次下载：

```bash
cd /home/r1lite
git clone <GIT_REPOSITORY_URL> duojin_ws
cd /home/r1lite/duojin_ws
./scripts/build_robot.sh
```

如果使用私有仓库，建议在工控机上配置只读 SSH deploy key，不要把 token 写入项目
文件。

## 工控机后续更新

```bash
cd /home/r1lite/duojin_ws
./scripts/update_robot.sh
```

`update_robot.sh` 只允许 fast-forward 更新，不会自动 merge，也不会覆盖工控机上的未提交
改动。

## 当前结构

```text
duojin_ws/
├── src/
│   └── duojin_arm_test/       # R1 Lite 末端位姿→IK→关节跟踪验证
├── scripts/
│   ├── build_robot.sh         # 在工控机上构建
│   ├── start_arm_environment.sh # 一键启动厂商控制环境
│   ├── run_arm_ik_test.sh     # 在工控机上运行测试节点
│   ├── update_robot.sh        # git pull --ff-only 并重新构建
│   └── deploy_to_robot.sh     # 无 Git 远程时的 rsync 备用方式
└── .gitignore                     # 不同步 build/install/log
```

详细真机流程见
[`src/duojin_arm_test/README.md`](src/duojin_arm_test/README.md)。
