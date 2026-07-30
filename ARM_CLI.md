# R1 Lite 机械臂终端 CLI 使用说明

`arm` 是 `duojin_robot_interface` 提供的终端入口。它统一调用项目的公开 ROS 2
Action：查询当前末端坐标、绝对/相对末端移动，以及六关节绝对角度目标。

关键规则：**不带 `--execute` 的移动命令也会真的向服务端发送 preview Goal。** 服务端会
检查反馈是否新鲜、TF、关节/末端范围、IK、控制权和 SDK 链路，但不会发布机械臂运动目标。
所以 `PREVIEW` 是服务端校验成功，不是 CLI 在本地猜测“应该可以”。

## 运行位置与前置条件

以下命令全部在 **R1 Lite 工控机**执行；其中 `~` 是
`/home/r1lite`，不是本地开发机的目录。

1. 机器人周围清场，底盘制动；真机执行时操作员持急停。软件检查不能替代急停和现场观察。
2. 工控机应只运行一份厂商 SDK，且没有 VR、Gello、teleop 或其他程序抢占机械臂目标。
3. 拉取本分支代码并构建 overlay。首次使用 CLI 或更新 CLI 后必须构建一次：

   ```bash
   cd ~/duojin_ws
   git pull --ff-only origin agent/arm-motion-api
   ./scripts/build_robot.sh
   ```

4. 选择一种启动模式：

   ```bash
   # preview 模式：可查询和校验，但绝不会真正执行
   ./start.sh

   # 真机验证模式：只有清场、制动并握住急停时才能使用
   ./start.sh --enable-arm-motion
   ```

   `start.sh` 会启动唯一的机械臂 API server，并停止已知会抢占关节目标的 EHI
   gateway。启动完成后会进入 `[duojin]` Shell；在这个 Shell 可直接运行下文的 `arm`
   命令。不要同时运行两次 `start.sh`。

若从另一个新终端执行命令，先加载相同环境：

```bash
source ~/galaxea/install_430/setup.bash
source ~/duojin_ws/install/setup.bash
```

两个终端还必须使用相同的 `ROS_DOMAIN_ID`。最简单的做法是直接使用 `start.sh` 打开的
`[duojin]` Shell。

## 先确认连接

```bash
arm --help
arm status left
arm status right
```

`arm status left` 是一次性查询，输出 `xyz=(x, y, z) m`、坐标系、时间戳和（若可读到）
六个关节角。当前对外末端坐标默认是 `base_link` 坐标系，单位为米；关节角单位为弧度。

持续显示当前末端坐标使用：

```bash
ros2 run duojin_robot_interface arm_pose_display --ros-args -p arm:=left
```

按 `Ctrl-C` 退出显示程序；它只读反馈，不控制机械臂。

## 推荐验证顺序

先在 `./start.sh` 的 preview 模式中运行每条命令（不加 `--execute`）。看到 `PREVIEW`
说明服务端完成了真实校验且没有发布机械臂目标。确认目标和环境后，关闭 SDK 并以
`./start.sh --enable-arm-motion` 重启；再把**同一条**命令加上 `--execute`。

`--execute` 只是 Goal 的第二道许可。若 server 是由默认 `./start.sh` 启动，带
`--execute` 的命令会被拒绝为 `EXECUTION_DISABLED`，不会运动。

### 相对移动（建议首次尝试）

相对增量单位为 m；默认沿 `base_link` 的 X/Y/Z 轴移动，并保持当前末端朝向。先从
1 cm 的小幅 Z 正向位移开始：

```bash
# 服务端 preview：不运动
arm shift left --dx 0 --dy 0 --dz 0.01

# 仅在 --enable-arm-motion 启动、现场已确认时执行
arm shift left --dx 0 --dy 0 --dz 0.01 --execute
```

也可指定增量轴所在坐标系：

```bash
arm shift left --dx 0.01 --dy 0 --dz 0 --frame base_link
```

不要未经 preview 就把较大增量或未知 frame 加上 `--execute`。

### 绝对末端坐标移动

绝对坐标的三个数字是所选 frame 内的目标位置，单位为 m，不是相对增量。先读取当前值，
自行把一个坐标只改动很小的量，再 preview：

```bash
arm status left

# 将下面 x/y/z 替换为 status 输出附近、已确认的目标值
arm move left --x <x_m> --y <y_m> --z <z_m> --frame base_link

# 同一目标通过 preview 后，才可在执行模式下运行
arm move left --x <x_m> --y <y_m> --z <z_m> --frame base_link --execute
```

例如 status 的 Z 是 `0.3331 m` 时，不要把任意示例坐标照抄；应基于当前坐标形成明确的
小幅目标，并先看 preview 结果。CLI v1 始终保持当前末端朝向，不能通过命令修改姿态。

### 关节目标

关节命令要求同时给出 6 个绝对角度（rad）。先从 `arm status` 复制当前关节值，只改动一个
关节不超过 `0.15 rad`，并使用较低速度：

```bash
# 替换 j1...j6 为当前附近的实际值；先 preview
arm joint left \
  --j1 <rad> --j2 <rad> --j3 <rad> \
  --j4 <rad> --j5 <rad> --j6 <rad> \
  --speed 0.20

# 同一命令的真机执行形式
arm joint left \
  --j1 <rad> --j2 <rad> --j3 <rad> \
  --j4 <rad> --j5 <rad> --j6 <rad> \
  --speed 0.20 --execute
```

`--speed` 取值范围是 `(0, 1]`；`0.20` 是推荐的首次验证值，不是碰撞规划或硬件安全保证。

## 返回结果与常见问题

| 输出/失败 | 含义与下一步 |
| --- | --- |
| `PREVIEW: ... no motion target was published` | 服务端已完成预览校验；没有运动。若目标正确，重启为执行模式后再加 `--execute`。 |
| `OK: ... final_xyz=...` 或 `joint goal completed` | 执行模式下服务端收到了符合到位条件的反馈；记录结果并再次观察实际机器人。 |
| `EXECUTION_DISABLED` | server 由默认 preview 模式启动；这是预期保护。使用 `./stop.sh` 后以 `./start.sh --enable-arm-motion` 重启。 |
| `CONTROL_CONFLICT` | 发现非本 API 的机械臂目标发布者（此前真机证据包含 EHI gateway）。停止冲突程序，再完整 `./stop.sh` / `./start.sh ...` 重启，不要绕过检查。 |
| `TF_UNAVAILABLE`、`FEEDBACK_STALE`、`SDK_NOT_READY` | 坐标转换、反馈或厂商执行链未就绪。先运行 `arm status left`，再检查 `start.sh` 输出和 ROS 图。 |
| `FAIL ... execution state is UNKNOWN` | 不要发送下一条 Goal；立即观察机器人，必要时使用硬件急停。 |
| `arm: command not found` | 尚未构建或当前终端没有 source overlay。运行 `./scripts/build_robot.sh`，然后重新启动或 source 两个 setup 文件。 |

命令可用参数以实时安装版本为准：

```bash
arm move --help
arm shift --help
arm joint --help
```

## 本次验证边界

本代码的 CLI 行为已完成本地 ROS-free 自动测试；工控机必须完成构建和 preview 验证后，才可
进行上述单臂、小范围的实际执行。当前说明不把 CLI 的 `OK` 当成碰撞安全或比赛级验收；每次
真机测试仍应记录目标、现场条件、命令、反馈和结果。
