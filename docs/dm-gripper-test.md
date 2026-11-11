# DM4310 夹爪测试与角度读取

本机通过以太网 SDK 直连控制器。左右均为角度增大闭合、角度减小张开，已标定张开 0 rad、闭合 0.94 rad，并接入[整体任务](../README.md)。独立脚本用于空载测试，退出会失能卸力；与整体任务或其他 SDK 控制程序互斥运行。

## 绝对角度开合

在项目根目录执行：

```bash
python3 scripts/gripper_test.py close --side left --position-rad 0.94
python3 scripts/gripper_test.py open --side left --position-rad 0
```

右侧用 `--side right`，两侧同角度用 `--side both`。`--position-rad` 是绝对目标，重复执行不累加；闭合目标应高于当前反馈，张开目标应低于当前反馈，方向相反会拒绝。配置目标范围 0–1.6 rad，不代表整个范围已验证为机械可用行程；脚本不自动寻找端点或判断夹到物体。

目标按限速发送完毕，实际位置和速度持续满足到位条件后完成，默认再保持 2 s（可用 `--hold-seconds` 修改）。终端显示起始、目标、失能前实际角度及退出结果；结束、失败或 Ctrl-C 均发送 FD 并等待新鲜 Off。失能后机构可能回弹。结果默认保存为 `runs/gripper-test-侧别-动作.json`，同名覆盖，`--result` 可另存。

## 手动开合时读取角度

```bash
python3 scripts/gripper_watch.py
python3 scripts/gripper_watch.py --side left
python3 scripts/gripper_watch.py --rate-hz 5 --csv runs/gripper-angles.csv
```

等待提示“所选夹爪均已确认 Off，可空载手动开合”后再手动改变开口。终端显示实际电机角度（rad、°），不是开口毫米数；读取不会改写零位或按控制范围截断反馈。

脚本默认 10 Hz 周期发送 FD 失能帧，并要求各侧新的 Off 应答后才显示；不发送使能或位置目标。反馈中断、失能无法确认时退出，不复用旧角度。`--rate-hz` 范围 1–50 Hz，`--samples 10` 读取 10 组后退出；实际频率受 SDK 收发影响。CSV 同名覆盖，记录 UTC 时间、侧别、角度、速度、力矩和状态。等待反馈时 Ctrl-C 最多可能延迟至启动超时（默认 3 s）后退出。

## 配置与判定

参数位于 [config/gripper-can.json](../config/gripper-can.json)，控制器及 SDK 路径位于 [config/default.json](../config/default.json)。

| 参数 | 当前值 |
| --- | --- |
| 左右张开／闭合 | 0／0.94 rad |
| 位置范围／目标限速 | 0–1.6 rad／0.2 rad/s |
| Kp／Kd／名义控制频率 | 3／0.12／100 Hz |
| 到位位置／速度容差 | 0.08 rad／0.3 rad/s |
| 到位稳定时间 | 0.1 s |
| 整体任务保持偏差／持续时间 | 0.15 rad／0.5 s |
| 反馈／启动／动作超时 | 0.5／3／15 s |

保持阶段各侧独立累计新反馈的超差时间，恢复到容差内清零，允许短暂波动。故障、非 On 状态、反馈失联仍失败；实际角度长期远离目标会动作超时，堵转不自动视为夹持完成。Python 周期控制不提供硬实时保证。

独立测试退出会失能；整体任务退出仅停止发帧并释放连接，不自动松爪或失能。停止软件周期保持不等于电机已物理失能。

## 链路与协议

SDK 链路为本工具 → Gento Terminal CANFD → 控制器末端 CAN → DM4310，不依赖 ROS 2 或 vcan。左右为 arm0／arm1，通道 1，Slave ID `0x01/0x02`，Master ID `0x11/0x12`。

Terminal 报文为 **4 字节小端 CAN ID + 8 字节数据**，不能直接使用 Linux SocketCAN 的 16 字节帧。FC 为使能、FD 为失能；本工具不提供 FE 调零、参数写入或模式切换。反馈状态 `0=Off`、`1=On`，其他状态按故障处理；接收同时检查 CAN ID 和帧内电机 ID。原生接收接口保留返回码，`-10` 为空轮询；SDK Python 包装会吞掉非正返回值，不能据其空结果区分所有错误。

依据：[DM CAN 协议](https://gento-teleoperation-apex.github.io/Apex_doc_center/end-effectors/dm-can-protocol)、[厂商 SDK CAN 桥源码](https://github.com/Gento-Teleoperation-Apex/gento_ros2_packages/blob/e6696cae3d95d84d1c14d0028ee267620bad9053/marvin_ros2_control/src/marvin_robot.cpp#L747)。当前验证结论见[验证记录](validation.md)。

## 底层诊断命令

```bash
# 只检查配置，不连接 SDK
python3 -m skye_task gripper inspect --transport sdk --robot-config config/default.json

# 被动接收，不发送控制帧；设备不主动反馈时可能超时
python3 -m skye_task gripper status --transport sdk --robot-config config/default.json --side left

# 明确失能卸力
python3 -m skye_task gripper disable --transport sdk --robot-config config/default.json --side both
```

`step --delta-rad` 支持单次不超过 0.1 rad 的相对小步；`move --position-rad` 为绝对移动；`open/close` 省略目标时使用配置角度。底层运动命令仅加 `--disable-on-exit` 才退出失能，详见 `python3 -m skye_task gripper --help`。底层结果默认 `runs/gripper-latest.json`，保留失能前及清理后反馈。

独立脚本固定使用 SDK；底层 CLI 默认 SocketCAN，当前直连环境须显式选 `--transport sdk`。SocketCAN 仅适用于已有 Apex／末端桥的环境，需停用其他夹爪命令源；创建同名 vcan 接口本身不能连通实机。程序内文件锁不能排除上位机等其他控制源。
