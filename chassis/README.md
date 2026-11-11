# chassis：底盘导航与精确停靠

运行在 Gento Skye 底盘主板（S100，用户 `sunrise`）上的停靠系统：导航到起停点 → 激光对正桌边／传送带边 → 带手臂／托盘碰撞检查的直线进场；ROS 2 桥接（`/gento_dock/cmd`、`/gento_dock/status`）；开机定位纠正。

- 安装：在底盘上执行 `bash install.sh`（安装到 `~/gento_dock`，不会动车）。
- 任务侧客户端：[`client/gento_navigation.py`](client/gento_navigation.py)，实现 `skye_task` 的 `Navigation` 协议。
- 离线测试：在仓库根目录执行 `python3 -m unittest discover -s chassis/tests -v`。
- 完整说明：[docs/chassis-navigation.md](../docs/chassis-navigation.md)。
