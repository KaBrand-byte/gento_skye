# 交接说明

更新日期：2026-11-11。运行入口见 [README](README.md)，点位与执行约定见 [方案](方案.md)，重要问题及验证范围见 [验证记录](docs/validation.md)。

## 当前状态

固定底盘双臂、分步升降及双爪动作已实现。当前接盘位 J1=±85°，用户已完成一轮整体流程。旧接盘路径的碰撞结论保留在验证记录中；程序无在线碰撞检测。

左右夹爪张开 0 rad、闭合 0.94 rad，判定允许 0.08 rad 到位误差及短时保持波动。双臂与夹爪共用 SDK 连接，独立测试／角度读取工具见 [夹爪说明](docs/dm-gripper-test.md)。导航、夹持／放置检测及完整任务成功率测试待完成。

## 设备与 SDK

| 项目 | 当前配置 |
| --- | --- |
| 上身控制器／SDK 与固件版本 | `6.6.7.190`／`00040701` |
| 本机网卡／地址 | `eno1`／`6.6.7.99/24` |
| 外部 SDK 路径 | `/home/gento/robot_sdk/tianji-robot-SDK`；`GENTO_SDK_ROOT` 优先于 JSON |
| 底盘 SSH | `sunrise@6.6.7.6` |
| Orin SSH | `nvidia@6.6.7.100` |
| 上身 SSH | `root@6.6.7.190` |

当前双臂与夹爪通过以太网 SDK 直连，不依赖 ROS 2 或 vcan。SDK 轨迹时基默认配置 50 Hz，实际设备频率不能通过当前接口查询。

## 后续导航接入

| 配置项 | 当前值 |
| --- | --- |
| ROS 2 接口环境 | `/home/gento/ros2_ws/interface_workspace/install/setup.bash` |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| `CYCLONEDDS_URI` | `file:///home/gento/.config/cyclonedds/cyclonedds.xml` |
| `ROS_DOMAIN_ID` | `24` |

使用导航时先加载上述环境文件，再设置对应变量。按 [interfaces.py](skye_task/interfaces.py) 实现任务下发、到站且停稳反馈及停止；完成后验证带盘运输、导航／操作精度及任务成功率。原始指标及厂家底盘接口资料保留在 [项目文件](项目文件/)。
