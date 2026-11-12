# 底盘导航与精确停靠

更新日期：2026-11-12。当前进度、部署状态和待办见 [handoff.md](../handoff.md)。本文说明底盘导航与精确停靠系统，以及它和 `skye_task` 的融合方式，包括部署、测试和微调。底盘端代码在 [`chassis/`](../chassis)；任务侧客户端在 [`chassis/client/gento_navigation.py`](../chassis/client/gento_navigation.py)，它实现 `skye_task` 的 `Navigation` 协议，但不依赖、也不修改 `skye_task`。

## 1. 结构

```
skye_task（运行在接入机器人内网的主机上）
   │  Navigation 协议：start / is_arrived_and_stopped / stop
   │  GentoNavigation：transport="ros"（默认）或 "ssh"
   ▼
/gento_dock/cmd ── std_msgs/String(JSON) ──▶ dock_bridge.py（底盘，常驻服务）
/gento_dock/status ◀── 状态、命令回执、地图位姿，2 Hz，transient local
   │
   ▼
run.sh → job_wrap.py → dock.py（底盘，后台任务，状态写入 job_status.json）
   │  厂家 ROS 2 接口（ROS_DOMAIN_ID=24，rmw_cyclonedds_cpp）
   ▼
灵猴底盘：导航任务、相对移动、激光点云、定位
```

- **一切运动都在底盘本机执行**。任务侧只负责下发任务、读取状态，网络中断不会让底盘处于半受控状态。
- **任务侧不需要厂家消息包**：桥接只用 `std_msgs/String`，任务侧只需要 rclpy 和 std_msgs。SSH 方式则只需要免密登录。
- **底盘端用到的厂家接口**：
  - `server/CreateTaskById`、`server/CreateTaskByPose`：按站台 ID 或位姿导航；
  - `server/CancelAllTask`：取消导航任务；
  - `move/MoveRelativeCmd`：相对移动；
  - `move/SetControlMode`、`move/SetParkingMode`：切换自动/手动、关闭驻车；
  - `move/State`：底盘状态；
  - `slam/LocationData`、`slam/Relocation`：读取定位、重新定位；
  - `/driver/lidar/point_cloud/Data`：三维点云（已在 base_link 坐标系）；
  - `driver/radar/Data`：超声波。
  
  另外，在底盘本机上调用两个网页后端接口：`/api/Task/GetAllTask`、`/api/Station/Navigation/GetAllStation`。

## 2. 一次停靠（`dock.py dock <站点>`）

站点有两种：**激光对边站点**（例如 D：对准传送带侧面）和**地图站点**（例如 T：前方没有可对齐的直边，`mode: map`）。两种站点都可以有一个人工示教的**安全点**（`staging_pose`，能带着双臂和托盘原地转身的位置）和一份**参照扫描**（`refs/<站点>.npy`）。

1. **出发前检查**：硬开关必须在自动；扫描与地图的匹配度低于 0.6 直接中止（定位明显错误时不导航）。
2. **离开当前站点**（`--leave`）：
   - 在地图站点上：沿进入时的直角路径平移回它的安全点；
   - 在激光站点正前方（例如 `undock` 之后，车在 D 的进场线上）：平移回该站点的安全点（e5806ed）；
   - 仍然不能原地转身时：找最短的纯后退或纯横移（必要时先后退再横移），确认移动本身无碰撞、移动后能转身，再执行。
3. **出发前转向**（e5806ed）：如果原地转身对双臂和托盘是安全的，先自己转向目的地，再交给厂家导航；不安全就跳过，由厂家导航自己转。
4. **导航到安全点**：调用厂家导航（`CreateTaskByPose`，有安全点时；否则按厂家站台 `CreateTaskById`），速度 `GENTO_NAV_SPEED`（默认 0.35 m/s）。
   - 停在安全点 0.35 m 以内且静止 1.5 s 即视为到达，并由脚本取消厂家仍挂着的任务（厂家停车偏差 5 cm 以上时会一直报 “Current Pos Without Node”）；
   - 离安全点 0.40 m 以外则中止。
   - 位姿任务不保证到站朝向：到站后原地转到站点的朝向（转身经过检查）。
5. **地图站点**：直角路径（先横后纵或先纵后横，选不碰的一条）平移到站点位姿；有参照扫描时随即用扫描匹配修正到约 2 cm / 1°（地图在 T 附近会跳约 10 cm，不再按地图修正）。已经站在该站点上时只做扫描修正。
6. **激光对边站点**：
   1. 拟合边沿（RANSAC + 全分辨率拟合，高度带由站点的 `z` 决定），把朝向校正到 `yaw_offset` 的 1° 以内（底盘不执行小于约 1° 的转身）。
   2. 横向位置沿边沿方向计算（原地转身不改变它），并计入之后直线前进时因残余朝向产生的漂移。参考点是边沿某一端；边沿被遮挡成几段时，用地图预测的位置选对那一段。
   3. 有参照扫描时：先纯横移到站点横线，再纯前进到距最终位置 0.30 m；重新测量；用参照扫描修正横向（小于 2.5 cm 时先多移 4 cm 再退回 4 cm，因为底盘不执行小于约 2 cm 的横移）和朝向；最后纯前进到位。
   4. 没有参照扫描时：按边沿端点横向对齐（1 cm），长距离同样分两段前进。
   5. 每一步移动前都用实时点云 + 机器人模型检查扫过的空间（直线余量 3 cm，转身余量 5 cm）；`accept` 记录的结构点（现场确认不会碰）在检查中忽略。放置站点还检查托盘正下方台面以上的空间，以及放置动作（前伸 0.10、下降 0.15）扫过的空间。台面高度只在最远的第一次测量时取一次（近处会把下方的框架层误当台面）。
7. **复测**：间隙、角度、横向误差和参照扫描残差写入 `FINAL`、`dock_log.csv`；累计前进距离写入 `/tmp/claude_dock_last.json`，供 `undock` 原路后退。

**安全**：厂家相对移动不会把“前进 + 横移”的组合指令走成直线（2026-11-12：x 0.75 / y 1.00 结束时转了 16°、偏 0.38 m，撞到传送带旁的桌子）。`rel()` 拒绝任何组合指令；所有移动都是纯前进、纯横移或原地转。

**设备特性**（实测，代码已处理）：小于约 1° 的转身不执行；小于约 2 cm 的横移不执行；直线前进固定少走约 11 mm（0.3 m 的短距离约 5 mm）；移动结束后定位还要约 1 s 才稳定。

**精度**（2026-11-12，参照扫描比对）：T 前后 0.9–6.6 mm、横向 2.8–7.4 mm、航向 ≤ 0.84°；D 横向 0.1–4.5 mm、前后 0.9–1.9 mm（补偿修正后）、航向 ≤ 0.88°。

## 3. 碰撞模型（`robot_model.py`）

- 模型只描述机器人自身，用车体坐标系里的长方体表示：底盘、躯干、上臂、手肘、双臂（按实测切片）、托盘包络。障碍物一律来自实时三维点云。
- 尺寸依据原厂三视图：底盘 676.7 × 547.1 mm，中心到前沿 338 mm；两肩相距 461 mm；上臂 287、前臂 314、腕加夹爪 165 mm。
- **双臂按实测**：`selfscan`（车不动）在运输姿态下用激光记录双臂（和托盘）的形状，沿前进方向每 5 cm 一片，记录该片的最低点和宽度；雷达只看得到右臂，左臂按镜像；没看到的片取相邻片中更低、更宽的值。结果写入 `robot_model.json` 的 `phases.carry_slices`。要求：运输姿态、夹着托盘、车前约 1 m 内无物体；手臂姿态改变后重做。**不要在任务里用 `--selfscan`**，它会在交接前（没有托盘时）覆盖模型。
- **手肘**：运输姿态下手肘在肩后约 0.23–0.35 m、离地 1.31–1.41 m（激光实测），单独建模（`elbow_back`、`elbow_z0`）。
- **托盘包络**：托盘在夹爪中的位置每次不同（后沿实测 0.33–0.45 m），模型用包络：车前 0.30–0.80 m、宽 ±0.22 m、离地 0.88–1.10 m（`robot_model.json` 的 `tray`）。
- 原地转身的检查按高度分层：每 1 cm 高度取能扫到的最大半径（当前最大约 0.88 m）。
- **框体与挂板**：D 左前方挂板边缘离手臂只有 1–2 cm（模型保守），现场确认不碰后已 `accept`。

## 4. 部署

### 底盘（S100，用户 sunrise）

```bash
# 把仓库的 chassis/ 拷到底盘，例如 scp -r chassis sunrise@6.6.7.6:~/gento_skye_chassis
bash ~/gento_skye_chassis/install.sh            # 安装到 ~/gento_dock，保留已有 stations.json / robot_model.json
```

`install.sh` 会：
- 复制脚本；
- 第一次安装时从 `stations.example.json` 生成 `stations.json`；
- 安装并启用以下 systemd 单元（需要免密 sudo）：
  - `gento-boot-check.timer`：开机 120 s 后清除残留任务，并检查、纠正定位；
  - `gento-dock-bridge.service` 和 `gento-dock-bridge.timer`：ROS 桥接。

整个安装过程不会让车动。

### 任务侧客户端

```bash
source /opt/ros/humble/setup.bash               # 或 handoff.md 中的接口环境
export ROS_DOMAIN_ID=24 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/gento/.config/cyclonedds/cyclonedds.xml   # Orin 上为 /etc/apex/cyclonedds.xml
python3 -c "import sys; sys.path.insert(0, 'chassis/client'); from gento_navigation import GentoNavigation; import time; n = GentoNavigation(); time.sleep(2); print(n.job_status(), n.pose())"   # 只读自检
```

- 在 Docker 容器中运行时，容器需要使用宿主机网络（`--network host`），否则 ROS 2 无法发现底盘。
- 离线测试：`python3 -m unittest discover -s chassis/tests -v`。

## 4.1 与 skye_task 融合（建议，本仓库未改动 skye_task）

`skye_task` 在“触发任务”和“前伸”之间预留了导航步骤（`Sequence.start()` 中记录 `navigation: skipped`），并在 `interfaces.Navigation` 中定义了协议。`GentoNavigation` 已按这个协议实现，接入只需要在 `skye_task` 侧做三处小改动，由维护者决定是否采用：

1. **配置**：增加一个嵌套的 `navigation` 配置段：`enabled`（默认 false）、`destination`（站点名，如 `"D"`）、`transport`（`"ros"` / `"ssh"`）、`gap_m`、`timeout_seconds`、`poll_seconds`。
2. **流程**：`Sequence` 接收一个可选的 `navigation` 对象。`enabled` 时，在原来记录 `navigation: skipped` 的位置执行以下步骤：
   - 调用 `start(destination)`；
   - 循环等待 `is_arrived_and_stopped()`，循环中每次都调用 `checked_feedback()`，检查停止请求、双臂和固定轴，并调用 `raise_if_failed()`，超过超时时间则报错；
   - 到站后确认双臂仍在接盘（运输）姿态，再进入前伸。
   - 在 `fail()` 中，如果导航还在进行，先调用 `navigation.stop()`。
3. **入口**：`run` 在 `enabled` 时创建 `GentoNavigation`，传入 `Sequence`，结束时调用 `close()`。

参考实现及 11 项离线测试保存在本地分支 `chassis-navigation-sequence-proposal`（未推送），需要时可以单独提交 PR 供评审。

## 4.2 全部在机器人上运行（Orin）

2026-11-11 已在 Orin 上部署并验证（只读）：
- **运行环境**：Orin（aarch64，Python 3.10）上装有厂家 `gento-sdk` 4.7.1（`/usr/local/lib/libGentoSDK.so`）。SDK 包 `tianji-robot-SDK/dist_aarch64/` 自带 ARM64 版 `libGentoSDKPY.so`。
- **SDK 部署**：把 `PYTHON_SDK/GentoRobot.py` 和 ARM64 版 `libGentoSDKPY.so` 放到 `~/robot_sdk/tianji-robot-SDK/PYTHON_SDK/`，并设置 `GENTO_SDK_ROOT=~/robot_sdk/tianji-robot-SDK`（skye_task 中环境变量优先于配置文件）。
- **代码部署**：`~/gento_skye` 放本仓库 `chassis-navigation` 分支的代码，不含 `项目文件`。
- **验证结果**：
  - `skye_task` 原有 60 项测试、`chassis/tests` 都在 Orin 上通过；
  - `plan-check` 连接控制器成功：通信延迟 2 ms，控制器 `0x00040701`，DOF 为双臂各 7、头 2、腰 2、升降 1；它报失败只是因为升降在 500 mm，不在接盘高度；
  - Orin 通过 ROS 2 桥接调用底盘的 `job_status`、`pose`、`check` 都正常。

一键运行（任务在 Orin 的 tmux 会话里运行，Wi-Fi 断了也不会中断）：

```bash
bash ~/gento_skye/chassis/client/orin_task.sh check            # 只读自检：配置 + SDK 版本
bash ~/gento_skye/chassis/client/orin_task.sh start           # 完整任务（会动）；--cycles 10 连续 10 次
bash ~/gento_skye/chassis/client/orin_task.sh attach           # 断线后重新接回
bash ~/gento_skye/chassis/client/orin_task.sh stop             # 软件停止：手臂 + 底盘
```

在 Windows 电脑上对应的命令是 `tools/gento.ps1 task | task-attach | task-stop | task-check`。

完整任务的顺序（`chassis/client/run_task_with_navigation.py`，不修改 `skye_task`）：
1. 读取双臂位置：已在接盘（运输）位则只连接、上使能，不动手臂；在准备位或过渡位则执行 `skye_task` 的 `prepare()`；其他姿态拒绝启动（`prepare()` 会直接回准备位，曾导致双臂相撞）。
2. 只读规划整个放置动作（前伸、下降、上移、后移）；下降超出双臂可达范围时，双臂下降到留 10 mm 余量的位置，其余由升降完成（当前：双臂 135 + 升降 15 = 150 mm）；不可行则在交接前停止。
3. 导航到 T；到 T 后才张爪，响铃提示递盘；按 Enter 后双爪闭合。
4. 导航到 D。
5. 放置：前伸 → 双臂下降（+ 升降补足）→ 松爪 → 升降回升 → 双臂上移 → 后移前伸的距离，直接回到接盘位。
6. 沿进入路线后退，导航回 T，等待下一个托盘。

参数：`--down-mm`、`--up-mm`（仅本次）、`--holding`（托盘已在夹爪中：跳过 T 和交接）、`--no-return`、`--cycles N`（连续 N 次，失败即停）。

结果写在 `~/gento_skye/runs/with-navigation.json`。

注意：
- 运行前确认 Apex 遥操作没有在控制手臂；目前只有 `apex-backend` 在运行。
- 充电位附近（紧贴虚拟墙、地图清理过）的扫描匹配不可靠，开机自检可能判定“定位不可靠”。出发前先把车推进工作区，再运行 `run.sh check` 或开机自检，确认定位正确后再开始。

## 4.3 在 Linux 上操作（Orin 本机或实验室 Ubuntu 电脑，不需要 Windows）

整个任务本来就在机器人上运行：导航与停靠在底盘上，任务程序（双臂 + 导航顺序）在 Orin 的 tmux 会话里。操作工具 [`tools/gento.sh`](../tools/gento.sh) 与 `gento.ps1` 命令相同（`task`、`task-attach`、`task-stop`、`goto D`、`relocate`、`status`……），会动车的命令都先要求输入 y。

**在 Orin 上直接用（推荐）**，一次性设置：

```bash
mkdir -p ~/.local/bin && ln -sf ~/gento_skye/tools/gento.sh ~/.local/bin/gento
gento setup          # Orin -> 底盘免密登录（只需输入一次底盘密码）
```

之后 `ssh nvidia@192.168.8.100`，运行 `gento task`（例如 `gento task --down-mm 120`、`gento task --holding`）。任务在 tmux 里运行，SSH 断开不影响；重新登录后 `gento task-attach` 回到任务窗口，`Ctrl-b d` 离开窗口而不停止任务。工具会自动识别在 Orin 上运行：任务在本机启动，底盘命令走有线网 `sunrise@6.6.7.6`。

**在实验室的 Ubuntu 电脑上用**：`~/.ssh/config` 加入

```
Host orin
  HostName 192.168.8.100
  User nvidia
Host chassis
  HostName 6.6.7.6
  User sunrise
  ProxyJump orin
```

再 `ssh-copy-id orin`、`ssh-copy-id chassis`（各输入一次密码），然后运行 `tools/gento.sh <命令>`；`gento.sh web` 通过 SSH 隧道打开底盘网页。

## 5. 运行与调试命令

在 Orin 上用 `gento <命令>`（`tools/gento.sh`），在 Windows 上用 `tools/gento.ps1 <命令>`，在底盘上直接用 `~/gento_dock/run.sh <命令>`：

| 命令 | 作用 | 是否动车 |
| --- | --- | --- |
| `status` / `job` | 状态、任务队列、日志；JSON 任务状态 | 否 |
| `relocate` | 扫描与整张地图匹配并重定位 | 否 |
| `selfscan` | 实测双臂（和托盘）形状并保存 | 否 |
| `check T` / `check D` | 报告从当前位置停靠会怎样执行 | 否 |
| `goto <站点>` | 导航并停靠（完整任务也走这个命令） | **是** |
| `lineup <站点>` | 同 goto，但在站点正前方对准后停下，不进场 | **是** |
| `undock` | 沿上一次进场路线直线后退 | **是** |
| `teach-pose <站点>` | 车在最终位姿：记录间隙、朝向、横向参考（地图站点只记位姿） | 否 |
| `teach-staging <站点> [map]` | 车在能转身的地方：记为安全点；`map` 表示按地图定位停靠 | 否 |
| `teach-ref <站点>` | 车在最终位姿：记录参照扫描 | 否 |
| `accept <站点> [clear]` | 车在站点正前方对准时：记录现场确认不会碰的结构 | 否 |
| `restage <站点>` | 在站点附近重新计算安全点（旧方式） | 否 |
| `tune <站点> gap\|lat_offset\|yaw_offset <值>` | 微调（写入 `stations.json`） | 否 |
| `task [参数]` / `task-attach` / `task-stop` / `task-check` | 完整任务（Orin，tmux） | `task` 会动 |
| `stop` | 软件急停：结束任务、取消移动和导航、切软手动 | 停车 |

动车前请确认：有人守着急停按钮，硬开关在自动，充电线已拔掉。

## 6. 微调流程

1. `goto T`（或由 `skye_task` 触发），到位后由上身执行写死的动作，观察偏差。
2. 按偏差调整：
   - 前后：`tune T gap 0.30`，单位 m；
   - 左右：`tune T lat_offset 0.02`，单位 m，正值表示车停得更靠左；
   - 朝向：`tune T yaw_offset 0.5`，单位度，正值表示车头更偏左。
3. 重复 `goto`，直到动作稳定。每次结果都追加在 `dock_log.csv` 里。

### 6.1 按最终位姿示教（推荐：T 交接点、D 放置点）

1. 手臂摆到运输（接盘）姿态，用遥控或网页把车开到**最终要停的位置和朝向**：T 是递托盘的位置，D 是手臂写死动作能把托盘放到传送带上的位置。
2. 运行 `teach-pose T`（或 `teach-pose D`），车不动。程序会：
   - 检查定位（扫描与地图的匹配度）；
   - 用激光拟合边沿，记录 `gap`（车头到边沿）、`yaw_offset`（相对正对边沿的朝向）、`end_y`（边沿左端或右端相对车中心的横向位置）；
   - 在地图上记录正后方 `BACK`（默认 0.5 m）处为导航点 `staging_pose`，同时记下最终位姿 `pose` 备查；
   - 提示模型余量：如果碰撞模型认为最终位姿已经贴到东西，停靠时会略早停下；放置站还会检查放置动作。
3. 停靠时：先导航到安全点并转向，然后按激光复现记录的朝向、横向位置和间隙，最后一段用参照扫描修正。最终位置以传送带和周围固定结构为基准，不依赖地图定位。
4. 若边沿那一端不在视野里（提示 `near the edge of the view`），换另一端：`teach-pose D 0.5 right`。

5. 安全点：推荐人工示教——把车开到附近一个空旷、能转身、到站点的直角路径上无障碍的位置，运行 `teach-staging <站点>`（这里不能转身会拒绝）。`teach-pose` 也会尝试自动寻找安全点；找不到时标记 `no_staging`，停靠会拒绝执行，直到示教了安全点。
6. 参照扫描：车在最终位姿时运行 `teach-ref <站点>`；站点周围布置变化后重新记录。
7. 紧挨手臂的结构：`lineup <站点>` 对准后运行 `accept <站点>`（必须在进场线上、距最终位置 0.25 m 以外运行；在最终位置上，模型内的结构看起来像机器人自身）。
8. 手臂模型：手臂姿态改变后，在空旷处、运输姿态、夹着托盘运行 `selfscan`（见第 3 节）。

### 6.2 按地图位置停靠的站点（例如 T 交接点）

站点前方没有可对齐的直边、或周围太挤时，用地图定位 + 参照扫描（约 1 cm）：

1. `teach-pose T`：车停在 T 的最终位姿（记录地图位姿）。
2. 把车开到附近一个**能安全转身**的点（拿着托盘也转得开，到 T 的直线路径上没有东西），运行 `teach-staging T map`（车不动；这里转不开会拒绝）。
3. 车回到 T：`teach-ref T` 记录参照扫描。
4. `goto T`：导航到安全点 → 原地转到 T 的朝向 → 直角路径平移到 T → 参照扫描修正到约 2 cm / 1°。离开 T 时先沿原路退回安全点再导航。

完整任务（`gento.ps1 task`）：T 交接 → Enter（双爪闭合）→ D → 放置（前伸、下降、松爪、上移，再后移前伸的距离，直接回到运输姿态）→ 沿进入路线后退 → 回到 T，双臂在运输姿态、双爪张开，等待下一个托盘（`--no-return` 跳过返回）。启动时双臂已在运输姿态则不动手臂、直接接入；在准备位或过渡位则执行 skye_task 的 `prepare()`；其他姿态拒绝启动（`prepare()` 会直接回准备位，曾导致双臂相撞）。

## 7. 已知问题与注意事项

- **开机定位会偏**：厂家开机时先用上次保存的位姿，校验不通过时再试“热点”候选位。但校验门槛太宽松（`init_score_thr=0.2`，错误候选的分数在 0.03–0.17 之间），在这个规整的房间里常常接受错误位姿，而且仍报告很高的置信度。
  - 现在由 `gento-boot-check` 用二维地图做扫描匹配来检查和纠正（`locmatch.py`）：匹配率不低于 0.85 才算正常；先在充电位附近比对，再全图搜索，结果有歧义时不纠正。
  - 热点文件中的厂家示例点已替换为本场地的点，原文件备份为 `hostpot_pose.pb.txt.bak_factory`。
- **任务会跨重启保留**：未完成的导航任务重启后会被重新创建，开关一切到自动就会继续执行。开机自检会先全部取消。
- **厂家避障高度上限是 0.8 m**（`wo.yaml` 中的 `avoid_range.z.max`）：行进途中，双臂和托盘高度上的障碍厂家避障看不到。目前只在转身前和进场前由本系统检查；要不要调高还待决定。
- **虚拟墙**：充电位紧挨着虚拟墙，从墙外侧无法规划路径，需要先把车推进允许区域。
- **两侧上料区选择安全侧**（任务指标 2）：还没实现。可以在 `stations.json` 中定义两侧的放置站点，由任务侧依次尝试，被挡则报错，再换另一侧。
- **厂家导航不知道双臂和托盘**：它按底盘外形避障，而双臂向前伸约 0.8 m、两侧约 0.47 m。验收时场地有障碍物，建议加大厂家导航的车体外形或膨胀半径，并调高避障高度上限。
- **组合移动已禁止**（见第 2 节）；如需提速，先实测小幅组合移动的轨迹，并增加运行中偏离保护。
- **双臂从接盘位直接去准备位会相撞**（`skye_task` 的 `prepare()`）：任务启动时会拒绝这种情况；双臂恢复用 Orin 上的 `~/gento_tmp/arms_to_ready.py`（经过渡位）。
- **上身重启或急停后**头、腰、升降可能长时间不上线（`-28 PDO data not init complete`）：松开所有急停后整机重启。
- **控制器同时只接受一个 SDK 连接**，断开后约 1 s 内拒绝新连接（`-22`）。
