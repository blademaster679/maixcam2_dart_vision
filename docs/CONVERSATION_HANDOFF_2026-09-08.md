# MaixCAM2 飞镖视觉项目：会话导出与跨对话交接

生成时间：2026-09-08（Asia/Shanghai）  
工作区：`/home/blade_master/pnx/maixcam2_dart_vision`
用途：将本文件上传到另一个对话，或让同一 Codex 工作区中的新对话先读取本文件。

> 说明：这是根据当前对话中可见的消息、仓库文档和 Git 状态整理出的结构化导出，
> 不是聊天界面逐字 transcript。它刻意区分了已验证事实、历史问题、尚未完成事项和
> 新对话的执行约束，避免把推测当成实测结论。

## 给新对话的启动指令

将本文件交给新对话后，可直接发送：

```text
请继续 MaixCAM2 RoboMaster 飞镖视觉项目。

工作区为 /home/blade_master/pnx/maixcam2_dart_vision。
请先完整阅读：
1. docs/CONVERSATION_HANDOFF_2026-09-08.md
2. README.md
3. docs/IMPLEMENTATION_STATUS_V0.2.md
4. 若任务涉及高速相机，再阅读 tools/os04a10_highfps/README.md 和相关验证报告。

先检查当前 Git 状态、现有代码、实测记录和设备状态，再汇报哪些工作已经完成、
哪些尚未完成。不要重复实现已完成事项，不要覆盖或丢弃现有未提交改动，不要把
无真值录像的输出率称为识别准确率，也不要把采集帧率称为检测帧率。

如文档与当前工作树冲突，以当前代码、原始测试证据和最新带日期报告为准，并明确说明差异。
```

## 项目目标与技术路线

项目用于 RoboMaster 比赛中的飞镖视觉制导。主要目标是：

- 远距离优先捕获微小绿色引导灯。
- 接近后同时识别目标颜色的左右装甲灯条。
- 输出绿灯中心、装甲中心、五个关键点、视线角、可选位姿和控制安全状态。
- 用状态机、轨迹跟踪和运动补偿提高短暂遮挡或漏检时的连续性，但低置信度结果必须
  `safe_for_control=false`。
- 后续可用局部 YOLO11n-Pose NPU 模型验证传统方法提出的 ROI；当前比赛模型尚不存在。

用户最初要求实现的 v0.2 混合方案为：

1. 原图多尺度高召回绿灯候选检测。
2. 红色或蓝色任意滚转双灯条检测与配对。
3. 绿灯、灯条与跟踪预测产生局部 ROI，交给关键点模型验证。
4. 归一化视线 Kalman、KLT/RANSAC 全局运动补偿和最多三条候选轨迹。
5. `SEARCH → ACQUIRING → TRACKING → COASTING → REACQUIRE` 状态机。
6. 统一 `TargetEstimate`、JSON schema v2 和 fail-closed 控制接口。
7. 当前纯视觉实现，预留未来 ICM-42688 `MotionPrior`。

原计划使用 640×480@60fps。板端优化和长测后，正式无模型实时档调整为
**480×360@60fps**；640×480 保留为画质优先档。

## 当前代码完成度

Git 基线：

```text
branch: main
HEAD: 0f3018e feat: implement v0.2 hybrid target guidance
origin/main: 0f3018e
```

`0f3018e` 已实现的核心内容包括：

- 归一化绿色响应、多尺度微小目标候选、捕获锥和预测 ROI。
- 最多五个绿灯候选，不再使用“越靠近画面中心越优先”的软先验。
- 红/蓝任意滚转双灯条几何、五关键点、装甲中心和可选 PnP。
- 归一化视线坐标跟踪、运动补偿、三候选轨迹与五级状态机。
- 预测控制最多两帧或 35ms，过期后继续报告但控制无效。
- `TargetEstimate`、JSON schema v2、旧字段兼容和未来 IMU 接口。
- NPU ROI 调度、模型加载与关键点结果融合接口。
- 回放评测、数据集处理、量化图选择、模型 staging 和相关单元测试。
- APP 构建、动态库路径修复、打包和板端运行说明。

当前明确没有完成、不能用占位结果代替的内容：

- 固定比赛镜头下的 480×360 内参和畸变实测标定。
- 曝光、增益和白平衡扫描及红蓝两套比赛配置。
- 本赛季目标灯条尺寸、灯条间距、绿灯相对装甲中心偏移的实测。
- 一万正 ROI、两万硬负 ROI 的人工标注数据集和独立第三场地测试集。
- 训练完成的 YOLO11n-Pose 权重、ONNX、AXMODEL 和 MUD。
- 带模型的板端 NPU P95、温度、30 分钟稳定性和降频测试。
- UART/CAN 传输层和控制台架闭环。
- 带真值的至少 300 次 15–25m 最终赛场验收。

因此当前默认应继续保持：

```ini
npu.enabled=false
target_geometry.pose_enabled=false
```

如果以后启用 NPU，建议同时设置 `npu.required=true`，加载失败或空间验证失败时必须
使 `safe_for_control=false`。

## 当前性能与录像验证结论

无模型板端结果：

- USB 有线、480×360 最坏负载：59.985 FPS。
- 传统检测 P95：15.17ms。
- 视觉运动补偿 P95：6.22ms。
- 总处理 P95：19.82ms。
- 正式配置连续 185 分钟：60.001 FPS、总处理 P95 17.37ms、估算丢帧率
  0.0072%，RSS 无增长。
- 同版本 640×480：59.38 FPS、总处理 P95 31.05ms，未通过原定 30ms 延迟门槛。

为了稳定 60fps，当前默认每两帧运行一次传统检测，计划跳过帧用 Kalman 更新，控制估计
仍按相机 60Hz 节拍输出；运动补偿和未来 NPU 也采用交错调度，不能将多个重负载阶段
简单串在每一帧上。

项目已有九段测试录像及多版检测可视化。重要限制：这些录像缺少逐帧目标真值和精确
距离标注，因此只能报告输出情况、轨迹连续性和性能，不能把“有效输出率”称为召回率、
准确率或赛场识别率。视频约 6 秒处曾出现目标丢失，这推动了全捕获锥重搜索、短时预测、
多轨迹和更稳健候选评分等改造；是否彻底解决仍需标注真值和独立测试确认。

## OS04A10 高帧率独立研发线

该研发线与现有绿色检测业务相互独立，当前没有把 360fps 采集接入
`main/src/green_detector_core.cpp`。

截至 2026-09-08 已验证：

- 逆向中心裁剪 RAW 路线：640×360，30 分钟 647,698 帧，平均 359.832fps，
  帧序号无异常。
- 官方模式 1344×760@180：全视野 NV21 通过 30 分钟采集。
- 官方 640×360@360：将 ITP 内部源队列从 1 增加到 4，输出队列保持 8 后，NV21
  通过 30 分钟采集；647,698 帧、平均 359.832fps、无缺号且无 MIPI 错误。
- 传感器原生 2688×1520 RAW 对照约 89.958fps。
- 640×360@360 的连续板端硬件编码尚未通过，编码器会出现队列积压；已有的一秒视频是
  先缓存 360 帧原始图像、再在主机编码，不能描述成持续硬件录像。
- 纯中心裁剪模式约覆盖原图宽、高各 24%；官方 binning 加裁剪模式约覆盖各 48%。
- “采集约 360fps”不代表当前检测算法可以逐帧处理 360fps。

OS04A10 相关代码和证据入口：

- `tools/os04a10_highfps/README.md`
- `tools/os04a10_highfps/REVERSE_ENGINEERING.md`
- `tools/os04a10_highfps/FULL_FOV.md`
- `tools/os04a10_highfps/OFFICIAL_VALIDATION.md`
- `tools/os04a10_highfps/OFFICIAL_RETEST.md`
- `artifacts/os04a10_official_20260908/`
- `artifacts/os04a10_retest_20260908/`

最新验证中使用的关键调用是：

```cpp
AX_VIN_SetPipeSourceDepth(0, AX_VIN_FRAME_SOURCE_ID_ITP, 4);
```

当前工作树含有尚未提交的 OS04A10 研发内容：

- `.gitignore` 已修改，新增忽略 `/.maixpy/`。
- `README.md` 已修改，更新高速研发线实测结论。
- `artifacts/`、`skills/`、`tools/os04a10_highfps/` 中存在未跟踪内容。

新对话开始工作前必须重新运行 `git status --short --branch`。这些改动属于用户当前工作，
不得擅自 `git reset --hard`、`git checkout --`、删除或覆盖。`__pycache__`、大视频和临时
运行数据通常不适合提交，需要在整理提交时逐项判断。

## 开发环境与板端连接

主机环境为 WSL2。用户先前使用 Windows 网络共享地址 `192.168.137.186`；之后改为 USB
有线连接，README 当前默认地址为 `10.18.197.1`。执行 SSH 前应以当前连接实测为准，
不要假设旧 IP 永久有效。

每次打开新的 WSL2 终端，环境变量不会自动继承，需要执行：

```bash
source ~/maix/maixcdk-venv/bin/activate
export MAIXCDK_PATH=~/maix/MaixCDK
export MAIXCAM2_HOST=10.18.197.1
cd /home/blade_master/pnx/maixcam2_dart_vision
```

比赛构建应固定与板端 MaixPy 对应的 MaixCDK commit，防止 API/ABI 随升级变化。已记录的
板端基线为：

```text
设备：MaixCAM2 + OS04D10
系统镜像：maixcam2-2026-05-29-maixpy-v4.12.5
MaixPy：4.12.5
MaixCDK：2a0502ecb20e5695b28580b3689492b7a228f9e4
工程：v0.2.0 / commit 0f3018e
```

板端查询传感器：

```bash
python3 -c 'from maix import camera; print(camera.get_device_name())'
```

WSL2 mirrored 网络模式曾出现 PowerShell SSH 正常、Linux `ssh -vvv` 停在
`fd 3 setting O_NONBLOCK` 的情况。当 Linux 路由异常时，可在 WSL 中调用 Windows
OpenSSH；USB 网络正常时优先直接使用 WSL 终端：

```bash
/mnt/c/Windows/System32/OpenSSH/ssh.exe root@"$MAIXCAM2_HOST"
```

## 构建、打包与上板关键命令

```bash
maixcdk build -p maixcam2
# 只修改现有源文件时可用：maixcdk build2

file build/dart_green_detect
maixcdk release -p maixcam2
unzip -l dist/dart_green_detect_v0.2.0.zip
```

`file` 必须显示 AArch64。正式安装：

```bash
scp dist/dart_green_detect_v0.2.0.zip root@"$MAIXCAM2_HOST":/root/
ssh root@"$MAIXCAM2_HOST" \
  '/maixapp/apps/app_store/app_store install /root/dart_green_detect_v0.2.0.zip'
ssh root@"$MAIXCAM2_HOST" \
  'cd /maixapp/apps/dart_green_detect && sh ./main.sh'
```

历史上遇到过以下问题：

- `maixtool` 安装成功，但 pip 报 `generate-parameter-library-py` 缺少 `typeguard`；这是
  环境中另一软件包的依赖冲突提示，不代表 `maixtool` 构建失败，仍应补齐依赖或隔离虚拟环境。
- `scp.exe` 直接接收 `\\wsl.localhost\...` 路径时曾只打印 usage；推荐直接从 WSL 执行
  `scp`，或先用 `wslpath -w` 转成 Windows 路径。
- 手工运行二进制曾报 `libax_sys.so` 找不到。根治方式已写入 `main.sh` 和打包结构：同时加入
  应用 `dl_lib`、`/opt/lib` 与系统库路径，不能用只包含 `./dl_lib` 的
  `LD_LIBRARY_PATH` 覆盖系统路径。
- 安装后曾出现二进制 `Permission denied`；需要确保打包产物保留执行权限，临时调试可
  `chmod +x dart_green_detect main.sh`。

## Git、录像和跨电脑开发

源码、配置、脚本、测试和小型文字报告适合进入 Git。原始录像、生成的检测视频、训练集、
运行日志、构建目录、模型二进制和发布 ZIP 通常不进入普通 Git 历史；大文件若必须版本化，
应使用制品库或评估 Git LFS 配额。

换电脑时，仅复制录像不足以继续完整开发，还需要：

- 克隆同一 Git commit 的源码和文档。
- 安装并固定 MaixCDK、交叉编译器、Python 虚拟环境和依赖。
- 单独复制被 Git 忽略的九段录像、必要的数据集、模型、安装包和验证证据。
- 记录相机型号、镜头、分辨率、曝光、固件、MaixPy 和 MaixCDK 版本。

## 后续工作优先级

比赛视觉主线：

1. 给九段录像补人工区间和逐帧真值，再运行强制验收，建立可信基线。
2. 固定镜头，在 480×360 下完成相机内参、畸变、曝光、增益和白平衡标定。
3. 测量本赛季目标几何，启用并验证 PnP 退化保护。
4. 建立五关键点数据集和第三场地锁定测试集。
5. 训练、量化、转换 YOLO11n-Pose，并先通过离线门槛。
6. 在板端验证带模型 60fps、延迟、温度、内存和降频。
7. 实现 UART/CAN 和控制台架 fail-closed 测试。
8. 完成至少 300 次有真值的 15–25m 赛场验收。

OS04A10 高速独立线：

1. 先整理并提交当前未提交代码、报告和适合版本化的小型证据。
2. 完成图像质量、Bayer/色卡、动态场景和多次冷/热启动验证。
3. 将已验证模式适配到公共 Camera/IVPS API，而不是仅停留在独立采集程序。
4. 解决或明确规避 360fps 持续硬件编码队列瓶颈。
5. 在高速模式下重新标定相机，并设计“采集线程只保留最新帧、检测线程按能力消费”的架构。
6. 最后再评估把高速相机接入绿灯识别主线；不要假设算法可处理每一帧。

## 本次对话的需求演进索引

以下是按时间整理的主要用户诉求，便于新对话理解为什么仓库形成现在的结构：

1. 查询 MaixCAM2 当前相机型号的方法。
2. 按官方教程编译 MaixCDK 程序并部署运行。
3. 解释为什么比赛版本要固定 MaixCDK commit，并把新终端环境步骤写入 README。
4. 解释 `maixtool` 安装时的 pip 依赖冲突。
5. 在工作区配置 `.clangd`。
6. 使用 `192.168.137.186` 上板，并给出编译后的上传、安装和运行命令。
7. 排查 WSL2 mirrored 模式下 SSH 卡在 `O_NONBLOCK`、而 PowerShell 可连接的问题。
8. 排查 APP 上传路径、安装包不存在、动态库缺失和执行权限问题，并做工程化修复。
9. 拉取板端录制视频，在本机叠加检测结果和 FPS，分析并改进效果。
10. 多轮拉取两段、三段、最终九段录像，执行最新版回放并总结结果。
11. 分析约 6 秒处目标丢失，优化识别率、重捕获和时序稳定性。
12. 整理 `.gitignore`、GitHub 上传边界，并解释 Git LFS 与限额。
13. 规划 640×480、15m 小绿灯、红蓝装甲双灯条、Pose NPU、PnP 和运动补偿方案。
14. 实现并补齐 v0.2 全部软件计划，明确外部数据和实机验收不可伪造。
15. 无模型回放验证、帧率优化和 USB 有线板端实测；最终选择 480×360@60fps。
16. 讨论为什么当前只有 60fps、OS04A10 能达到的高速模式以及驱动开发所需官方资料。
17. 编写请求官方技术支持的邮件，并解释寄存器表、时钟、MIPI、曝光和 SDK 资料的作用。
18. 将当时 Git 改动汇总为 `0f3018e`，更新 README 和下一步计划。
19. 继续开发 OS04A10 640×360@360fps，完成独立采集、官方模式复测和证据归档。
20. 当前请求：把上述上下文导出为文件，供另一个对话阅读。

## 权威状态入口

新对话不应仅依赖本文件的摘要，还应检查：

- `README.md`：日常使用、构建、部署、回放和下一步计划。
- `docs/IMPLEMENTATION_STATUS_V0.2.md`：v0.2 实现矩阵、门槛和不能伪造的外部产物。
- `reports/PERFORMANCE_OPTIMIZATION_2026-08-31.md`：无模型性能优化。
- `reports/DEVICE_BENCHMARK_2026-09-01.md`：板端实测。
- `tools/os04a10_highfps/README.md`：高速相机路线总览。
- `tools/os04a10_highfps/OFFICIAL_RETEST.md`：最新 360fps 采集与编码边界。
- `git status --short --branch` 和 `git log --oneline --decorate -12`：当前真实版本状态。

## 跨对话使用说明

在 Codex IDE 中打开同一工作区，新对话可以读取本文件和当前工作树，但不会自动获得本次
聊天的逐字记录。若只需要继续原聊天，应使用界面的“最近聊天”，或在 Codex CLI 中运行
`/resume` / `codex resume`。需要启动独立新对话时，应保留本文件作为显式上下文，并在
开始修改前检查 Git 状态。
