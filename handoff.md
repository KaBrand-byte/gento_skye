# 进度交接

更新日期：2026-11-11

## 当前方案

见[实施方案](方案.md)，包含控制接口、任务流程及待确定事项。

## 当前进度

- 已实现 `skye_task` Python 空载双臂顺序流程，运行与配置说明见 [README](README.md#双臂空载动作程序)。
- 已加入 SDK 0407 适配、MoveJ／双臂同步直线规划、坐标转换、固定姿态检查、停止与执行结果记录。
- 用户首次真机实验已通过连接、位置模式及准备位／接盘位 MoveJ；前伸 150 mm 在规划阶段返回 `-1020`，停止双臂成功，完整路径未通过。详见[首次实验记录](docs/experiments/2026-11-11-first-run.md)。
- 用户已更正准备／接盘关节示教位、腰头零位、升降 500／450 mm；前伸和下降均改为 100 mm，侧向退出各 100 mm。代码、默认 JSON 及方案已同步。
- 已实现双臂先到位、升降再调整并等待稳定的分步流程；直线动作期间升降固定在 450 mm，返回准备姿态后恢复 500 mm。升降调整期间停止会请求保持当前反馈位置。
- 更正后的点位已用本地原生 SDK 与出厂模型、零 TCP 通过准备→接盘及前伸→下降→退出→双臂回准备位的完整规划。见[更正后的离线验证](docs/experiments/2026-11-11-corrected-poses.md)。该结果不验证真机模型、碰撞或升降执行。
- 已改为按当前关节选择邻近逆解、增加轨迹首点连续性检查及失败前位姿记录；新增只读 `plan-check`，要求设备已处于 450 mm 接盘高度。
- 用户于 14:35–14:36 完成更正配置后的整轮真机空载流程，包括升降、前伸、下降、退出和回准备位。紧接着再次运行，在准备位上传 2 点保持轨迹时返回 `-19`。已根据 SDK 底层 5–5000 点限制修正静止轨迹及短轨迹处理，加入连续两轮回归测试；60 项离线测试通过，重复启动修复尚待真机复测。见[实验与修复记录](docs/experiments/2026-11-11-repeat-run.md)。
- 先前本工作环境只读连接返回 `-2`（ethernet work is down），助手离线定量复现基于 SDK 出厂参数；真机完整运行结果来自用户提供的日志。
- 首次真机运行前需核对控制器轨迹时基与配置频率（默认 50 Hz）；当前 SDK 上传接口不携带频率，也不能读取实际执行频率。
- 夹爪统一入口与导航反馈接口已预留；当前不发送夹爪指令、不模拟开合成功。
- 底盘导航暂未实现；本阶段预留导航接口，后续再接入导航功能。
- 尚未完成导航精度、操作精度及“端持—运输—放置”完整任务成功率测试。

## 其他说明

### ROS 2 / DDS 环境配置

| 配置项 | 当前值 |
| --- | --- |
| ROS 2 接口环境加载文件 | `/home/gento/ros2_ws/interface_workspace/install/setup.bash` |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| `CYCLONEDDS_URI` | `file:///home/gento/.config/cyclonedds/cyclonedds.xml` |
| `ROS_DOMAIN_ID` | `24` |

启动终端后，若接口环境加载文件存在，先通过 `source` 加载该文件，再设置上述环境变量。

### 运控 SDK 环境配置

| 配置项 | 当前值 |
| --- | --- |
| SDK 版本 | `0407` |
| `GENTO_SDK_ROOT` | `/home/gento/robot_sdk/tianji-robot-SDK` |

### SSH 连接

```sshconfig
# 底盘
Host chassis sunrise
    HostName 6.6.7.6
    User sunrise

# 域控（Orin）
Host orin
    HostName 6.6.7.100
    User nvidia

# 上身运控（x86 控制器）
Host upper root
    HostName 6.6.7.190
    User root
```
