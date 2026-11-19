# 演示当天检查单

厂家底盘已经出过两种问题，任意一种都会让演示失败：

| 问题 | 表现 | 出现过 |
|---|---|---|
| 厂家程序没起来或退出了（`linkhou_robot.service`，`Restart=no`，退出后不会自己重启） | ping、ssh 都正常，但激光、底盘状态、定位都没有数据；`gento relocate`、`gento goto` 报错 | 11-16、11-18 |
| 底盘连 Orin 的网口 `eth1` 掉线 | 掉线时 ping 时通时断；任务报“导航桥状态超时”；厂家定位可能因为 IMU 超时被重置 | 11-17（9 分钟内掉了 39 次） |

`gento doctor` 一条命令能把这几种情况分开，并告诉你该怎么处理。它不会动车，约 30～40 秒。

## 演示前（提前 30 分钟开始）

1. **整机断电重启一次。** 开机后等满 **3 分钟**，让厂家程序启动完。
2. **执行 `gento doctor`**，必须看到 `==== 总结论：可以开始 ====`。
   - 有 `[失败]` 时，按那一行箭头后面的提示处理，再执行一次 `gento doctor`。
   - `[警告]` 要看一下：例如“eth1 掉线 N 次”说明网口已经不稳，再断电重启一次。
3. 定位有问题时执行 `gento relocate`，然后再 `gento doctor`。
4. **完整演练一轮**：`gento task --handover`。跑完后看日志顶部：
   - 响应时间是否合格；
   - `chassis eth1 link changes during this run: 0`（掉线次数应为 0）。
5. 把车停回 T，准备好托盘。

## 演示期间

- 急停旁边一直要有人。网口掉线时，Orin 发出的软件停车指令到不了底盘。
- 不要在底盘上执行别的命令或诊断。底盘本来就很忙（4 核，负载 10～14）。
- 每两轮之间执行一次 `gento doctor`，大约 30 秒。

## 出问题时

| 看到的 | 怎么办 |
|---|---|
| `the vendor chassis programs are not running`，或体检显示“厂家主服务没在运行”“激光/底盘状态/定位没收到任何数据” | 整机断电重启，等 3 分钟，`gento doctor`，`gento relocate`，再重新跑。大约需要 5 分钟 |
| 任务里出现“底盘导航桥 … 无状态”的警告 | 这时网口可能在掉线，任务会等最多 60 秒。**底盘作业会自己跑完**，网络恢复后任务会读到结果，继续往下走。盯住车，必要时拍急停 |
| “底盘导航桥状态超时（… 60 s 无应答）” | 网络断了超过 60 秒。拍急停或 `gento stop`，然后 `gento doctor`；网口仍然掉线就整机断电重启 |
| “定位不对”“localisation looks wrong” | `gento relocate`，不行就在厂家网页上重定位 |
| “被挡住 … 按 Enter 重试” | 清掉障碍后按 Enter |

## 需要厂家处理或由你决定的

- 让厂家把 `linkhou_robot.service` 改成退出后自动重启，并说明它为什么会退出或没有起来（看 `journalctl -u linkhou_robot -b`）。
- 把 `eth1` 掉线的问题报给厂家：内核 `6.1.158-rt58`，驱动 `hobot_gmac` 加 RTL8211F，日志里有 `gmac_check_tx_fifo: tx queue:0 fifo is not empty`。
- 厂家的 DDS 配置 `/etc/ros/cyclonedds/cyclonedds.xml` 绑定在 `eth1` 上。网口一掉线，底盘内部的 ROS 通信可能也会一起受影响，这一点请厂家确认。
- 可以试着关掉网卡的 TSO：`sudo ethtool -K eth1 tso off`，重启后会失效。是否这样做由你决定。
