# Gento Skye 双臂托盘任务

使用 Python SDK 0407 控制固定底盘下的双臂、升降和 DM4310 夹爪，完成人工递盘、夹持、前伸、下降、松爪、上移和收臂。去接盘及返回准备位均经过过渡位，实际点位以加载的配置为准；关键验证结论见 [验证记录](docs/validation.md)。导航、在线碰撞检测及夹持／放置成功检测尚未实现。

## 运行

使用 Python 3.10+，项目本身只依赖标准库。真机另需外部 SDK 的 `PYTHON_SDK/GentoRobot.py` 及原生库，默认路径 `/home/gento/robot_sdk/tianji-robot-SDK`。可通过 `GENTO_SDK_ROOT` 或任务配置的 `sdk_root` 修改，环境变量优先；控制器默认 `6.6.7.190`。

在项目根目录执行：

```bash
# 离线检查配置，不加载 SDK、不连接设备
python3 -m skye_task check --config config/default.json

# 连接设备并执行
python3 -m skye_task run --config config/default.json --result runs/latest.json
```

`run` 连接并通过初始化检查后进入位置模式并运动。使用当前 `config/default.json` 时，流程如下：

1. 双臂到准备姿态，确认升降在准备高度 700 mm。
2. 双爪张开至 0 rad，双臂依次到过渡位、接盘位，每段到位稳定后继续，确认升降在接盘高度 700 mm。
3. 保持张爪等待人工递盘，按 Enter 后双爪闭合至 0.94 rad。
4. 同步前伸 100 mm、下降 150 mm；张爪后，双臂同步上移 150 mm。
5. 双臂依次返回过渡位、准备位，每段到位稳定后继续，确认升降在准备高度 700 mm。

每次开合到位后等待 1 s，闭爪后额外等待 1 s 供人工交接；Enter 后自动完成后续动作。双臂与升降分步执行，当前准备和接盘高度均为 700 mm，因此各段双臂运动期间升降均保持该高度。启动要求实际升降高度在 699～701 mm；高度不符时需先人工调整。过渡位通过任务配置的 `transition` 指定。

上移沿机器人参考系 +Z 执行并保持起始姿态，与下降方向相反；距离由 `up_mm` 配置（当前 JSON 为 150 mm），速度、加速度和频率由 `up` 配置。未加载配置文件时，Python 内置的升降高度为准备 500 mm、接盘 450 mm，下降和上移均为 100 mm。

Ctrl-C／SIGTERM 请求停止双臂；升降若正在调整则请求保持当前反馈位置。故障时终止流程，冻结夹爪已发目标，不自动松爪、失能或收臂。退出前停止夹爪周期线程，再释放 SDK；退出后不再有软件周期保持，不能据此判断电机已物理失能。

## 配置

| 文件 | 内容 |
| --- | --- |
| [config/default.json](config/default.json) | 点位、升降高度、运动参数、等待和超时；支持部分覆盖 |
| [config/gripper-can.json](config/gripper-can.json) | 双爪角度、CAN ID、控制参数及到位／保持容差 |
| [方案.md](方案.md) | 完整示教值、动作顺序及 SDK 执行约定 |

默认 `gripper_enabled=true`，运行命令可用 `--gripper-config 路径` 指定夹爪配置；设置 `gripper_enabled=false` 可执行双臂空载流程，跳过开合与交接等待。

左右夹爪均为角度增大闭合，张开 0 rad、闭合 0.94 rad。当前 JSON 的到位容差为 0.7 rad、速度容差 0.3 rad/s、稳定时间 0.1 s；保持阶段允许 0.7 rad 偏差，连续超差 0.5 s 才失败。反馈失联、电机故障和非 On 状态仍会停止流程。

轨迹规划及控制器时基配置默认为 50 Hz。SDK 无执行频率查询接口，设备实际时基需单独核对；规划成功和点位到位检查均不包含碰撞检测。

## 诊断与独立工具

只读规划从**当前反馈姿态**计算前伸、下降、上移、回过渡位和准备位，要求升降已在配置的接盘高度（当前 JSON 为 700 mm）、腰头符合固定姿态。此命令不检查准备→过渡→接盘路径，不移动到示教位，不切换模式，也不发送控制、停止或夹爪指令：

```bash
python3 -m skye_task plan-check --config config/default.json
```

独立夹爪工具用于空载标定，结束时会失能卸力；不得与整体任务或其他 SDK 控制程序同时运行：

```bash
python3 scripts/gripper_test.py close --side left --position-rad 0.94
python3 scripts/gripper_test.py open --side left --position-rad 0
python3 scripts/gripper_watch.py
```

右侧使用 `--side right`。实时读取脚本确认 Off 后允许手动开合；协议、参数和其他命令见 [夹爪测试说明](docs/dm-gripper-test.md)。整体任务与独立工具共用控制器锁，防止本项目内重复连接。

## 结果与验证

结果默认写入 `runs/latest.json`，包含各段目标／反馈、夹爪状态、版本、错误及停止信息。`motion_completed` 表示动作流程完成，`gripper_actions` 表示夹爪动作状态；`grasped`／`placed` 仍为 `null`，不代表已检测到夹持或放置成功。下次运行会覆盖同名结果，可用 `--result` 指定其他文件。

```bash
python3 -m unittest discover -s tests -q
```

离线测试覆盖动作顺序、连续运行、SDK 轨迹约束、坐标转换、到位与超时、停止、只读规划、夹爪协议及共享连接。测试使用替身，不连接设备；当前真机验证范围见 [验证记录](docs/validation.md)。

原始资料：[任务指标](项目文件/移动双臂机器人任务指标-2026年12月.md)、[厂商资料说明](<项目文件/Gento Skye资料/原厂资料说明.md>)。设备连接和后续导航入口见 [交接说明](handoff.md)。
