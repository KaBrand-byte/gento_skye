# 底盘导航与精确停靠

更新日期：2026-11-11。本文说明底盘导航与精确停靠系统，以及它和 `skye_task` 的融合方式，包括部署、测试和微调。底盘端代码在 [`chassis/`](../chassis)；任务侧客户端在 [`chassis/client/gento_navigation.py`](../chassis/client/gento_navigation.py)，它实现 `skye_task` 的 `Navigation` 协议，但不依赖、也不修改 `skye_task`。

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

1. **出发前检查**：硬开关必须处于自动，否则直接中止。厂家导航起步时会原地转身，而它的避障只看离地 0.8 m 以下的点。所以先用三维激光检查“底盘 + 躯干 + 双臂 + 托盘”原地转一圈会不会碰到东西。
   - 带 `--leave` 时，如果转身不安全，会先找一段最短的直线移动（后退、横移或斜后方），确认这段移动本身无碰撞、移动后可以安全转身，再执行。
   - 不带 `--leave` 时直接中止。
2. **导航到起停点**：调用厂家导航去站点的起停点（`staging` 站台）。
   - 车停在离起停点 20 cm 以内、静止 3 s，就视为已到达。厂家服务端在停车偏差约 5 cm 以上时会一直报 “Current Pos Without Node”，不结束任务，所以这种情况下由脚本自行取消该任务。
   - 车停在离起停点 25 cm 以外时中止（任务被取消或失败）。
3. **对正**：用激光拟合桌边或传送带外侧面，校正朝向，目标 0.4° 以内。
   - 相对移动的实测特性：转角固定少转约 0.77°，小于约 1° 的指令不执行；直线固定少走约 11 mm，重复性 ±1 mm；横移只能达到指令的约 75%。脚本都做了补偿或迭代。
4. **横向对齐**：以面上某一端的端点为基准，横移到示教位置，可再加 `lat_offset`。`untaught` 站点（例如尚未示教的放置点 D）跳过这一步，直接从当前横向位置进场。
5. **直线进场**：直线开到 `gap`（底盘前沿到边沿的距离）。进场前检查整个扫过的空间是否会碰撞（直线移动余量 3 cm，转身余量 5 cm）。
   - 带 `--closest` 时，如果被挡，就停在最近的安全位置，并记录原因。
   - 放置站点（`place: true`）还会检查托盘正下方、台面以上 2 cm 的空间是否为空。
6. **复测**：再测一次间隙、角度和横向误差，写入 `FINAL` 和 `dock_log.csv`。

历史实测（旧版流程）：直线进退的间隙误差在 ±1 mm 级；桌边一轮停靠的间隙误差 −6 mm、角度 0.68°；传送带一轮间隙误差 +2 mm、角度 −0.32°。2026-11-11 用新流程到桌边：间隙 0.447 m（目标 0.45 m），角度 0.27°。

## 3. 碰撞模型（`robot_model.py`）

- 模型只描述机器人自身，用车体坐标系里的若干长方体表示：底盘、躯干、上臂、前臂与夹爪、托盘。障碍物一律来自实时三维点云，场地里任何物体的位置都不写死。
- 尺寸依据原厂三视图：
  - 底盘 676.7 × 547.1 mm，中心到前沿 338 mm；
  - 两肩相距 461 mm，肩轴在底盘中心前方约 49 mm；
  - 上臂 287、前臂 314、腕加夹爪 165 mm；
  - 升降行程约 0.90 m。
- 双臂和托盘的实际占位用激光实测：`run.sh selfscan`，结果存入 `robot_model.json`。
  - **在运输姿态（接盘位，托盘在手）下、车前约 1 m 内没有其他物体时运行**；
  - 姿态或升降高度变了，要重新测；
  - 雷达通常只看得到一侧手臂，左右按较宽的一侧对称处理。
- 托盘默认尺寸可以在 `robot_model.json` 的 `tray` 中覆盖：前沿超出底盘 0.45 m，深 0.35 m，宽 0.45 m，底面离地 1.2 m。
- **托盘抬到 1.2 m 只能越过桌面（约 0.9 m）和皮带（约 0.84 m），越不过传送带上方的黑玻璃检测框（约 0.9–1.52 m）。** 放置点必须示教在托盘和双臂都避开框体的位置。原放置点距框边只有约 0.24 m，曾在原地转身时撞到框体。

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

## 5. 运行与调试命令

在底盘上（`~/gento_dock/run.sh …`），或在 Windows 电脑上用 [`tools/gento.ps1`](../tools/gento.ps1)：

| 命令 | 作用 | 是否动车 |
| --- | --- | --- |
| `status` / `job-status` | 状态、任务队列、日志；JSON 格式的任务状态 | 否 |
| `selfscan` | 用激光测双臂和托盘的占位并保存 | 否 |
| `check T` / `demo-dry` | 报告从当前位置停靠会怎样执行 | 否 |
| `goto <站点> [间隙]` | 导航并停靠到站点（`skye_task` 也走这个命令） | **是** |
| `to-start [间隙]` / `demo [间隙]` | 演示用：到桌边 T；从桌边到传送带 D | **是** |
| `tune <站点> gap\|lat_offset\|yaw_offset <值>` | 微调参数（写入 `stations.json`） | 否 |
| `teach <站点>` | 在起停点、正对边沿时，把当前横向位置记为该站点 | 小幅转身 |
| `stop` | 软件急停：结束任务、取消相对移动和导航、切到软手动 | 停车 |

动车前请确认：有人守着急停按钮，硬开关在自动，充电线已拔掉，托盘端在 1.2 m 以上。

## 6. 微调流程

1. `goto T`（或由 `skye_task` 触发），到位后由上身执行写死的动作，观察偏差。
2. 按偏差调整：
   - 前后：`tune T gap 0.30`，单位 m；
   - 左右：`tune T lat_offset 0.02`，单位 m，正值表示车停得更靠左；
   - 朝向：`tune T yaw_offset 0.5`，单位度，正值表示车头更偏左。
3. 重复 `goto`，直到动作稳定。每次结果都追加在 `dock_log.csv` 里。
4. 放置点 D：动作固定后，把车停到期望的横向位置、正对传送带，运行 `teach D`。示教后 `untaught` 标记会去掉，之前的微调值也会清空。

## 7. 已知问题与注意事项

- **开机定位会偏**：厂家开机时先用上次保存的位姿，校验不通过时再试“热点”候选位。但校验门槛太宽松（`init_score_thr=0.2`，错误候选的分数在 0.03–0.17 之间），在这个规整的房间里常常接受错误位姿，而且仍报告很高的置信度。
  - 现在由 `gento-boot-check` 用二维地图做扫描匹配来检查和纠正（`locmatch.py`）：匹配率不低于 0.85 才算正常；先在充电位附近比对，再全图搜索，结果有歧义时不纠正。
  - 热点文件中的厂家示例点已替换为本场地的点，原文件备份为 `hostpot_pose.pb.txt.bak_factory`。
- **任务会跨重启保留**：未完成的导航任务重启后会被重新创建，开关一切到自动就会继续执行。开机自检会先全部取消。
- **厂家避障高度上限是 0.8 m**（`wo.yaml` 中的 `avoid_range.z.max`）：行进途中，双臂和托盘高度上的障碍厂家避障看不到。目前只在转身前和进场前由本系统检查；要不要调高还待决定。
- **虚拟墙**：充电位紧挨着虚拟墙，从墙外侧无法规划路径，需要先把车推进允许区域。
- **两侧上料区选择安全侧**（任务指标 2）：还没实现。可以在 `stations.json` 中定义两侧的放置站点，由任务侧依次尝试，被挡则报错，再换另一侧。
- 写死动作的放置点 D 还没示教；`demo` 和 `goto D` 目前从当前横向位置直线进场。
