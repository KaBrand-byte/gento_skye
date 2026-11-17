# 4090 上位机部署与无机器人测试

用户于2026-11-13授权开始分步部署与测试。只在上位机独立目录安装模型环境，本阶段不加载机器人SDK、不切换控制模式、不发送运动指令。

## 已核实的主机

| 项目 | SSH只读实测 |
| --- | --- |
| 登录目标 | `gento@192.168.8.101`，主机名Gento |
| 系统 | Ubuntu 22.04.5 LTS，x86_64 |
| CPU | Intel Core i9-14900KF |
| GPU | RTX 4090，24564 MiB显存，驱动595.99.02 |
| 盘点时空闲显存 | 23264 MiB，会随桌面进程变化 |
| 内存 | 62 GiB，盘点时约53 GiB可用 |
| 磁盘 | 系统分区约1.4 TB可用 |
| 网络 | Wi-Fi `192.168.8.101/24`；有线 `6.6.7.99/24` |
| 原有Python | 3.10.12，不替换 |

## 安装位置与版本

全部部署文件位于 `/home/gento/vla_pi05`：

```text
tools/bin/uv           用户目录下的uv，不修改shell启动文件
python/                uv管理的Python 3.11
openpi/                官方源码及项目.venv
cache/uv/              依赖缓存
cache/openpi/          官方检查点及分词器缓存
cache/huggingface/     HF缓存
bootstrap.sh           隔离安装及GPU基础检查
env.sh                 当前进程环境变量
download_http.py       分段续传与官方检查点校验
official_smoke.py      回环策略服务器、随机观测测试
run_smoke.sh           顺序执行下载和推理通信测试
logs/                  安装、模型服务及测试结果
```

官方源码固定到 `215abfb217dbac7d5f1273282331b9b1866c0479`；依赖按该提交的uv.lock安装。uv安装器来自Astral官方地址，本次版本为0.13.8。CUDA运行库随Python依赖安装；不修改系统驱动或安装系统CUDA。

2026-11-13 已完成安装，`logs/bootstrap.exit` 为0。JAX 0.5.3识别 `cuda:0`，BF16 GPU矩阵乘法通过；PyTorch 2.7.1+cu126识别RTX 4090。模型及分词器已复制到上位机并校验。官方π0.5-DROID简短推理及Orin真实观测远程推理均已通过，见下方实测记录。

## 在Windows下载并手动复制

按用户要求，检查点改在当前Windows电脑下载，保留现有代理，使用六路并发。下载目录：

```text
C:\Users\98623\Documents\gento\pi05_download
```

下载器固定每个对象的GCS generation，以32 MiB分段续传；20个对象共约11.576 GiB，逐个核对大小和官方MD5。未完成内容保存在 `pi05_droid.partial`；全部校验通过后才改名为 `pi05_droid`。另已下载并校验官方 `paligemma_tokenizer.model`。Windows下载完成后，将整个 `pi05_download\cache\openpi` 文件夹复制到上位机 `/home/gento/vla_pi05/cache/`，最终结构应为：

```text
/home/gento/vla_pi05/cache/openpi/
  openpi-assets/checkpoints/pi05_droid/params/
  big_vision/paligemma_tokenizer.model
```

避免产生 `cache/openpi/openpi/` 的额外嵌套。上位机已有的 `.gcsfs-incomplete` 目录是先前中断的下载，不能作为完整模型使用。复制后核对文件校验值，再开始下面的模型测试。

本地查看进度使用 `pi05_download/logs/download.log` 和 `download.exit`；最终退出码0才表示本次下载完成。可在WSL复核完整检查点：

```bash
python3 gento_skye/vla/deploy/download_http.py --root "$PWD/pi05_download" --verify-only
```

## 查看进度

在上位机终端执行：

```bash
tail -n 40 ~/vla_pi05/logs/bootstrap.log
cat ~/vla_pi05/logs/bootstrap.exit
tail -n 40 ~/vla_pi05/logs/smoke.log
tail -n 40 ~/vla_pi05/logs/policy-server.log
cat ~/vla_pi05/logs/smoke-result.json
```

尚未生成的日志或退出状态文件表示对应步骤未结束或尚未开始，应结合实际进程判断。退出码0才代表对应脚本成功。

## 本轮测试含义

1. 在新环境用JAX执行GPU矩阵乘法，并确认PyTorch识别CUDA。
2. 取得官方 `gs://openpi-assets/checkpoints/pi05_droid`，按桶文件清单校验20个对象的大小和MD5，约11.6 GiB。
3. 用官方policy_config加载pi05-DROID，启动仅监听127.0.0.1的WebSocket策略服务。
4. 官方客户端发送make_droid_example生成的随机观测，预热2次，再测2次请求；可用 `VLA_SMOKE_STEPS` 修改正式请求数。
5. 从固定版本配置读取动作块长度，本次为15；检查返回动作是有限数值的15×8数组，记录请求时延和GPU显存占用，结束后停止本次测试服务。

这里的15是动作块长度，8是DROID单臂动作维数，不能与Skye的16维双臂动作混淆。简短测试成功证明官方模型能在4090上加载与推理、回环通信可用；真实相机链路由下面的独立部署测试验证。

## 已部署的真实观测链路

```text
Orin CSI0/4/5 → 1280×720拼图 → 显式裁剪三视图、等比补边至224×224 RGB
Orin SDK0407 → 左右臂各7关节（度转弧度）及被动夹爪反馈
             → 官方WebSocket协议 → 4090 6.6.7.99:8000
             → pi05_droid诊断 → 15×8动作块 → Orin JSONL记录
```

4090文件位于 `/home/gento/vla_pi05`；Orin文件位于 `/home/nvidia/vla_pi05`。新增两个用户级服务，均已启动、未设置开机启动：

| 机器 | 服务 | 用途 |
| --- | --- | --- |
| 4090 | `skye-vla-policy.service` | 加载模型并接收观测，与SSH会话独立 |
| Orin | `skye-vla-camera.service` | 使用独立完整参数文件启动三路相机 |

在对应机器以登录用户执行 `systemctl --user status 服务名` 或 `systemctl --user stop 服务名`。重启机器后，Orin先按原有方式执行 `cd ~/cam_geac && sudo ./rb_camera.sh init`，再启动相机服务；4090启动策略服务：

```bash
# 4090
systemctl --user start skye-vla-policy.service

# Orin
systemctl --user start skye-vla-camera.service
bash ~/vla_pi05/run_observation.sh --requests 5 \
    --output ~/vla_pi05/runs/live-$(date +%Y%m%d-%H%M%S)
```

Orin轻量依赖限定在 `~/vla_pi05/client_python`：msgpack 1.1.2、typing_extensions 4.12.2、websockets 15.0.1，以及固定提交的官方openpi_client源码。复用系统ROS、numpy、OpenCV和Pillow。SDK仅在有界观测程序运行期间连接，结束后释放；不启动Apex Robot/Teleop、不启用或切换控制模式、不发送CAN或运动命令。使用原Apex相机流程前先停止 `skye-vla-camera.service`，避免同时占用相机。

当前权重是DROID单臂策略。接收端保留三视图和双臂14关节，但模型本身仅用头部、左腕及左臂7关节。夹爪暂无被动反馈，原始记录保持null；当前诊断服务显式使用左夹爪0 rad作为测试输入，并在每个响应写入 `diagnostic_assumptions` 和 `measured:false`。未启用这个诊断选项时，缺少左夹爪反馈会拒绝请求。全部响应固定 `execution_supported:false`，动作保持DROID语义，不能作为Skye双臂目标。

要继续双臂任务，需要匹配Skye的16维动作、训练归一化统计及权重，并接入实际夹爪反馈。当前没有把单臂输出补维为双臂动作，也没有实现机器人执行接口。

## 2026-11-13实测结果

- 上位机20个检查点对象MD5校验通过；官方简短测试2次正式请求74.49、82.79 ms，显存9289 MiB。
- 有线Orin→4090 ping平均0.687 ms。
- 真实观测推理5次成功，图像ROS时间戳和机器人RT帧号均更新5次。首次8.64 s（含JAX编译）；之后92.61、85.77、82.31、82.60 ms，平均85.82 ms。这里计时范围为发送已预处理观测至收到动作，采集和预处理另计。
- 发送时图像ROS时间戳年龄24.52～70.84 ms；拼图回调至状态读取15.29～55.01 ms。拼图没有提供每个相机独立的采集时间戳，尚不能证明严格同步。
- 本次SDK被动轮询两夹爪均无CAN反馈，记录为缺失；运动指令数0。Apex Robot/Teleop服务保持inactive。
- 新增输入映射、夹爪缺失处理和不可执行响应的4项测试通过。

本地证据：`runs/vla-deploy-20261113/inference/smoke-result.json` 和同目录 `live-infer-20261113-2120/`（摘要、逐请求状态/动作、原拼图与三路224×224输入）。Orin对应目录为 `~/vla_pi05/runs/live-infer-20261113-2120/`。

## Orin侧已经完成的独立观察

使用原厂 `~/cam_geac/rb_camera.sh init` 初始化相机后，通过独立相机节点收到CSI 0、4、5的头部及双腕画面；CSI 1关闭。8.02秒内收到243张具有不同ROS时间戳的JPEG拼图，约30.18 Hz。本地证据为 `runs/vla-deploy-20261113/cameras/mosaic.jpg`。该结果证明相机链路可取图，尚未与策略输入整合。

另完成约1.99秒、183次被动SDK状态读取；未启用机器人、未切换控制模式、未发送运动指令。夹爪位置尚未取到，图像与机器人状态尚未同步。这些观察不代表双臂策略适配完成。

## 复跑

基础安装成功后，在上位机执行：

```bash
bash ~/vla_pi05/run_smoke.sh > ~/vla_pi05/logs/smoke.log 2>&1
```

此命令会重写本轮smoke测试日志和结果；如需保留旧结果，先复制 `logs/`。下载缓存会复用。当前阶段不提供机器人执行命令。

依据：[官方固定版本README](https://github.com/Physical-Intelligence/openpi/blob/215abfb217dbac7d5f1273282331b9b1866c0479/README.md)、[DROID输入输出变换](https://github.com/Physical-Intelligence/openpi/blob/215abfb217dbac7d5f1273282331b9b1866c0479/src/openpi/policies/droid_policy.py)。
