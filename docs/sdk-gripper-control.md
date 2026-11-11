# SDK 夹爪控制调研

核对日期：2026-11-11。仅阅读文档和源码，未连接机器人、未发送夹爪指令。

用户已确认实装设备为天机自研夹爪，接在机械臂末端。已确认 SDK 提供末端通信通道，但资料没有提供该夹爪的具体通道及开合协议，尚不能确定开合命令字节或开度、速度、夹持力参数。

## 资料与版本

- [1004 版本运动 SDK 文档](https://alidocs.dingtalk.com/i/p/WO4GjWeoDKJeQzDR/docs/XPwkYGxZV3vZ11axUrb9BqgKWAgozOKL)：已读取线上正文，末端通信与灵巧手数据接口的说明与本地 SDK 一致。
- 本地 SDK：`/home/gento/robot_sdk/tianji-robot-SDK`，核对 `PYTHON_SDK/GentoRobot.py`、`C_SDK/L1Robot/L1Robot.h` 及对应 C++ 实现。
- 本机 SDK 配置记录为 4.7.1（`00040701`，0407 分支），本次未重新查询控制器。文档标题中的“1004”不能作为 SDK 的版本号；SDK 与控制器的 MAJOR/MINOR 必须一致，连接版本不兼容返回 `-4`。
- MoveJ 与双臂同步 MoveL 示例负责机械臂运动；夹爪开合使用独立的末端设备通信接口。

## 接口选择

| 设备接入方式 | Python 接口 | 数据含义 |
| --- | --- | --- |
| 机械臂末端 CAN FD／RS485 外设 | `terminal_set()`、`terminal_get()`、`terminal_clear()` | 每次最多 64 字节的外设通信数据，开合报文由夹爪协议定义 |
| 控制器支持的末端手爪／灵巧手模块 | `hand_get_type()`、`hand_set_data()`、`hand_get_data()` | 每个末端 512 字节的数据块，字段布局依赖该设备的协议 |

对象使用 `FXObjType.OBJ_ARM0`（左臂，0）和 `FXObjType.OBJ_ARM1`（右臂，1）。这些接口没有双爪掩码参数；双爪需分别调用，接口不承诺两爪在同一时刻动作。

### 末端 RS485／CAN FD

Python 封装位置：`GentoRobot.py:2292` 起；C 接口位置：`L1Robot.h:836` 起。

```python
robot.terminal_clear(obj_type)  # 返回 0 表示成功
ret, send_ms = robot.terminal_set(obj_type, chn_type, data, timeout=1000)
channel, response, recv_ms = robot.terminal_get(obj_type, timeout=1000)
```

- 通道枚举：`FXChnType.CHN_CANFD = 1`、`CHN_485A = 2`、`CHN_485B = 3`。A／B 表示同一末端的通信通道，左右臂由 `obj_type` 指定。
- `data` 接受 `bytes` 或十六进制字符串，最多 64 字节；超时单位为 ms。
- `terminal_set()` 的 `ret == 0` 表示 SDK 发送操作成功，仍需校验设备应答和动作状态，才能判断开合完成。
- `terminal_get()` 返回三元组中的首项是通道号。当前 Python 封装将底层所有 `n <= 0` 情况统一转为 `(0, b"", 0)`，因此无法从这个返回值区分接收超时和其他底层错误；若需要精确诊断，可通过 ctypes 直接调用 C 接口并保留返回码。
- `terminal_clear()` 清除终端通信缓冲，不是夹爪复位。适合通信初始化时使用，不应在等待当前事务应答时清空。
- SDK 的这些收发函数只检查连接和目标对象，没有要求将机械臂切到 Position 模式；夹爪本身的上电、初始化和使能要求仍取决于夹爪协议。

可核对的厂商示例是 `C_EXAMPLE/example_basic_Terminal.cpp`：连接 → 清缓冲 → 向 ARM0 的 RS485A 发送 → 接收并检查通道。示例使用串口工具回显 `00 01 02 03 04 05 06 07`，该数据不是夹爪开合命令。

以下仅展示已连接后的通信模板，`command` 必须由实际夹爪协议生成：

```python
from PYTHON_SDK.GentoRobot import FXObjType, FXChnType

def exchange_gripper_frame(robot, obj_type, channel, command: bytes):
    if not command or len(command) > 64:
        raise ValueError("夹爪报文长度必须为 1–64 字节")
    ret, send_ms = robot.terminal_set(obj_type, channel, command, timeout=1000)
    if ret != 0:
        raise RuntimeError(f"末端发送失败: {ret}")
    received_channel, response, recv_ms = robot.terminal_get(obj_type, timeout=1000)
    if not response:
        raise RuntimeError("未收到末端应答，需进一步区分超时与通信错误")
    if received_channel != channel:
        raise RuntimeError(f"收到其他通道数据: {received_channel}")
    # 调用方还需按夹爪协议处理分包／组帧、校验地址、功能码、校验和及状态。
    return response

# 仅在接线和协议确认后调用，例如左臂、RS485A：
# response = exchange_gripper_frame(
#     robot, FXObjType.OBJ_ARM0, FXChnType.CHN_485A, command
# )
```

若夹爪采用 Modbus RTU，需要按其寄存器表构造地址、功能码、数据及 CRC；SDK 没有自动构造这些内容。当前资料也未给出实装设备的波特率、校验位、站号和寄存器表，不能假定默认值。

### Hand 数据接口

补充核对本地 `Platform-V1004天机机器人上位机软件操作手册20261015.pptx`：第 27 页明确将 `Hand Data Communication` 描述为面向“手爪／灵巧手”的 512 字节原始报文工具。因此，实装设备是夹爪，并不能单凭接口名称排除 `hand_set_data()`。具体是否适用于天机自研夹爪，仍需厂商确认。

Python 封装位置：`GentoRobot.py:1339` 起。

```python
ret, hand_type = robot.hand_get_type(obj_type)
ret, serial, feedback = robot.hand_get_data(obj_type)
ret = robot.hand_set_data(obj_type, command_data)
```

- 类型枚举包含 `FX_HAND_NOT_USED`、`FX_HAND_TJDH_6`、`FX_HAND_TJDH_21`、`FX_HAND_DEXBOT`、`FX_HAND_STARERA`，没有明确的通用两指夹爪类型。
- `hand_get_type()` 返回的是控制器配置的手类型；返回 `NOT_USED` 不能据此判定末端没有通过 RS485 接入夹爪。
- `hand_get_data()` 返回错误码、反馈序列号、512 字节数据。成功读取缓存不等于反馈正在更新，判断新反馈应同时检查序列号。
- `hand_set_data()` 会把短数据补零到 512 字节、将长数据截断至 512 字节。实际使用应先按协议验证长度和字段，避免无意补零／截断。
- 两份资料均未给出这些 512 字节中使能、位置、力和状态等字段的布局。
- 线上更新历史中出现的旧 `FX_L1_Runtime_SetHandAction/SetHandPos/SetHandP/SetHandD/SetHandMaxTor` 不在当前本地 `L1Robot.h` 和 Python 封装中，不能直接照旧接口调用。

## 接入本项目所需的信息

品牌及安装位置已确认：天机自研夹爪、机械臂末端。下一步需确认具体型号／协议版本、SDK 接口、供电／通信参数，以及初始化、使能、开合和状态查询协议。还需明确开度、速度、夹持力范围及单位，以及“到位”“夹到物体”“故障”的反馈含义。

厂商上位机提供两个相关入口，不能仅凭末端接线位置判断该用哪一个：

- `More Features → CAN/485`：手册第 18 页说明 CAN／COM1／COM2，COM1、COM2 分别对应 485 CH1、CH2。源码 `FX_PLATFORM/UI.py:3258` 调用 `terminal_set()`。
- `More Features → Hand Data Communication`：手册第 27 页说明固定 512 字节数据。源码 `FX_PLATFORM/UI.py:4486` 调用 `hand_set_data()`。

上位机源码中 CAN／485 指令列表初始化为空，未找到天机夹爪预置开合命令；不能把通用收发窗口当成已实现的夹爪控制面板。

可向天机直接索取：这台 Skye 的自研末端夹爪在 0407 SDK 下应调用 `terminal_set()` 还是 `hand_set_data()`？若用前者，请提供 CAN／485 通道和完整报文；若用后者，请提供 512 字节字段布局。两种情况均需初始化、开合、状态查询示例以及反馈含义。

确认协议后，可将对应报文封装进 `skye_task/interfaces.py` 预留的 `Gripper.control(target, action)`，分别支持左爪、右爪和双爪。开合完成与夹持成功应分别判断；双爪其中一侧失败时不能返回整体成功。
