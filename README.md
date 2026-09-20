# MaixCAM2 飞镖绿灯与装甲板制导 v0.2

本工程面向 RoboMaster 飞镖视觉。比赛主模式按 **OS04A10 全视野1344×760@180fps** 开发，
已接入独立VIN异步业务入口；标定和真实目标验收完成前控制保持无效。
保留 `480×360@60fps` 稳定配置作为回退基线；`640×480` 为画质优先档。v0.2 使用
“捕获锥/预测 ROI 多尺度绿灯候选 + 任意滚转双灯条几何 + 局部 YOLO11n-Pose +
视线坐标跟踪/全局运动补偿”的混合链路，并通过统一 `TargetEstimate` JSON v2
输出控制安全状态。

代码和工具链已经实现；比赛模型、相机标定、目标实物尺寸和 300 次实机验收不是
可以由代码自动生成的占位数据。当前默认配置因此保持
`npu.enabled=false`、`target_geometry.pose_enabled=false`。完整完成度和仍需实测的
项目见 [v0.2 实施状态](docs/IMPLEMENTATION_STATUS_V0.2.md)。基于全视野 180 fps 代码、
实测证据和捕获时序重新审查后的当前方案见
[全视野 180 fps 绿灯识别更新方案](docs/GREEN_DETECTION_PLAN_FULL180.md)；后续开发以该文档
为准，下面的 60 fps 步骤只保留为历史基线。

## 项目目录与本地资料

| 目录 | 用途 | Git 管理 |
| --- | --- | --- |
| `main/`、`config/` | 检测、跟踪、融合、输出与运行配置 | 保留源码和配置 |
| `tests/` | 主机侧 C++ 单元测试 | 保留测试代码 |
| `tools/os04a10_highfps/`、`tools/video_replay/` | 高速采集、驱动实验与离线回放 | 保留工具、测试和技术文档 |
| `tools/offline_recorder/` | 无电脑现场录制 APP、打包与拉取工具 | 保留源码和使用说明 |
| `tools/imu_icm42688/`、`tools/wifi_diagnostics/` | IMU 采样、零漂评估与无线诊断 | 保留脚本、测试和使用说明 |
| `tools/dataset/`、`tools/model/`、`models/` | 数据集处理、模型训练和部署契约 | 保留脚本和模板，忽略数据与模型产物 |
| `docs/`、`skills/` | 设计方案、实施状态与研发流程 | 保留文档，忽略下载的 PDF 规格书 |
| `reports/`、`recordings/`、`artifacts/` | 测试报告、录像、测量摘要和图表 | 仅本地保存，保留 `artifacts/README.md` 管理说明 |

本文及技术文档指向 `reports/`、`recordings/`、`artifacts/` 和 `.maixpy/` 的链接为本地
证据路径，新克隆的仓库不含这些材料。复现实测或交接证据时需另行复制对应批次资料。

## 480×360稳定链路（既有v0.2）

- 在原始 480×360 RGB 图上组合 `2G-R-B`、绿色占比、局部背景均值/方差、亮度与
  饱和白芯，并用 `2/4/6/9/14 px` 多尺度响应搜索微小灯点。SEARCH/REACQUIRE
  扫描 ±6.5° 捕获锥，锁定后扫描预测点周围 72 px ROI；实际漏检立即回到全锥。
- 每次传统检测记录最多 5 个绿灯候选；全图 LAB 路径仍能留下锥外调试候选，但任何
  锥外候选都不能进入控制。
- 不再使用“越靠近画面中心分数越高”的软先验。内部最多维护 3 条候选假设，以
  外观、尺度、运动残差和视觉全局运动更新。
- 主跟踪器在归一化视线坐标中估计视线、角速度、尺度和尺度变化率；KLT 风格稀疏
  光流加 RANSAC 相似变换补偿相机扫动和滚转。
- 状态固定为 `SEARCH → ACQUIRING → TRACKING → COASTING → REACQUIRE`。
  预测最多 2 帧或 35 ms 可用于控制，之后仍可报告但
  `safe_for_control=false`；丢失后立即恢复全捕获锥搜索。
- 按 `armor.expected_color=red|blue` 检测任意角度细长灯条，配对检查平行度、长度、
  色彩响应一致性、间距以及与绿灯的相对几何。`auto` 只允许调试，并强制控制无效。
- 五关键点顺序固定为绿灯、左灯条两端、右灯条两端。端点完整且分离充分时，可用
  带畸变处理的平面 PnP 求位姿；重投影超限时位姿无效。
- 接近阶段连续 3 次装甲几何有效后，在 100 ms 内从绿灯平滑切到装甲中心。
- NPU 只处理 `clamp(8×绿灯直径, 64, 384)` 的候选 ROI。搜索时轮询前三候选，
  跟踪时优先当前轨迹；启用模型后传统检测与 NPU 自动交错运行，各约 30 Hz，
  Kalman/控制输出保持 60 Hz，避免把两段延迟叠在同一帧。

## 480×360回退基线的帧率结果

默认配置在每两帧执行一次传统检测（30 Hz），计划跳过帧由 Kalman 更新，控制估计仍按
相机的 60 Hz 节拍输出。全捕获锥每 30 次传统检测刷新一次，真实漏检立即恢复全锥；
全局运动补偿也为 30 Hz，并且默认仅在目标锁定后启用。终端 JSON 降到 10 Hz 且不
序列化候选数组，避免逐行 `flush` 限制相机循环。

USB 有线板端最坏负载实测中，480×360 达到 **59.985 FPS**，传统检测 P95 为
15.17 ms、视觉运动补偿 P95 为 6.22 ms、总处理 P95 为 19.82 ms，估算丢帧为 0；
同一版本的 640×480 为 59.38 FPS、总处理 P95 31.05 ms，未通过 30 ms 门槛。
正式配置又连续运行 185 分钟，达到 60.001 FPS、总处理 P95 17.37 ms，估算丢帧率
0.0072%，RSS 无增长。因此正式档选择 480×360。九段无模型回放和完整限制见
[帧率优化报告](reports/PERFORMANCE_OPTIMIZATION_2026-08-31.md)和
[板端实测](reports/DEVICE_BENCHMARK_2026-09-01.md)。录像缺少逐帧真值，输出率不能
解释为真实召回率或误报率；长测没有芯片温度字段，温度上限仍需单独记录。

## 原v0.2执行计划（2026-09-02，历史基线）

以下计划保留原60fps基线口径；比赛主模式的标定、性能与数据验收应另以1344×760@180配置完成。下面只列尚未完成的工作。v0.2 检测、跟踪、JSON、回放评测、模型接口和无模型
60 FPS 优化已经完成，不再重复开发；每一步只有达到“完成条件”后才进入依赖它的下一步。

最近一次板端实测的复现基线为 MaixCAM2 + OS04D10、系统镜像
`maixcam2-2026-05-29-maixpy-v4.12.5`、MaixPy `4.12.5`、MaixCDK
`2a0502ecb20e5695b28580b3689492b7a228f9e4` 和本工程 `v0.2.0`。二进制、配置与安装包
哈希记录在[实施状态](docs/IMPLEMENTATION_STATUS_V0.2.md)；升级其中任一项后必须建立新的
基线记录，不能沿用本次性能结论。

| 步骤 | 工作和产物 | 完成条件 |
| --- | --- | --- |
| 1. 固化软件基线 | Git 归档源码、配置、测试和工具，实测报告单独保存；保留上述版本和哈希记录；提交后还需将 v0.1.5/v0.2 安装包保存到团队制品库 | 新电脑可按本文重新构建；Git 中不含录像、训练集、运行日志和模型二进制；制品库可取回两个安装包 |
| 2. 核验九段现有录像 | 人工标注目标/无目标区间，并为关键帧补充绿灯中心、五关键点、颜色和可见性；用 `evaluate_replay.py --enforce` 生成有真值的基线 | 九段录像不再只报告“输出率”；得到捕获延迟、控制级误报、候选召回、关键点和视线误差 |
| 3. 完成 480×360 相机标定 | 固定比赛镜头与安装姿态，标定内参/畸变；扫描 100/200/400/800/1600 μs、增益和白平衡，分别保存红蓝比赛配置 | 25 m 绿灯仍可见、运动拖影不超过 1.5 px；重投影误差和参数版本有记录 |
| 4. 完成目标几何标定 | 测量本赛季灯条长度、两灯条中心距、绿灯相对装甲中心偏移，填入 `target_geometry.*`；分别验证红、蓝和全滚转 | 完整灯条时 PnP 重投影 P95 ≤2 px；退化情况不输出有效位姿 |
| 5. 建立模型数据集 | 按完整录像和场地采集/划分数据，人工标注五关键点；复核视频 0/1 的 LED 屏硬负样本，并补齐距离、颜色、滚转、模糊和无目标场景 | ≥1 万正 ROI、≥2 万硬负 ROI；第三场地锁定测试；相邻帧不跨集合 |
| 6. 训练并转换 Pose 模型 | 训练 YOLO11n-Pose，比较 192/256/320 输入；导出固定输入 ONNX，选 100 张代表图做 INT8 校准，通过 Pulsar2 生成 NPU2/NPU1 AXMODEL 和 MUD | 锁定测试集正候选召回 ≥99.5%、关键点 P95 ≤2 px，且控制级误报为 0 |
| 7. 完成 NPU 板端验收 | 仅替换 `models/runtime/` 产物并启用已经实现的 NPU 路径，重新打包；测 30 分钟 FPS、端到端/NPU P95、丢帧、RSS、温度和降频 | 持续 60 FPS；端到端 P95 ≤30 ms、NPU P95 ≤20 ms、丢帧 <0.1%，无内存增长或热失效 |
| 8. 接入控制与台架闭环 | 为同一个 `TargetEstimate` 增加 UART/CAN 序列化、序号、时间同步、校验和和超时失效；先做录制数据回灌和台架硬件在环 | 控制端只接受 `safe_for_control=true`；断流、陈旧预测、模型失败和错误颜色均能在限定时间内安全失效 |
| 9. 完成赛场验收 | 在 15/20/25 m、±5°、红蓝、全滚转、不同光照和等效运动下执行至少 300 次独立测试，锁定配置后禁止用测试集继续调参 | 达到本文“验收原则”的捕获率、有效率、最长失效间隔、误差和零控制级误报要求 |

步骤 2～4 可以并行准备，但步骤 6 必须等待步骤 5 的数据集冻结，步骤 7 必须等待模型
通过离线锁定测试，步骤 8～9 必须使用已经通过板端验收的同一二进制、模型和配置。

### OS04A10 360 FPS 独立研发线

2026-09-07：已从现有OS04A10源码与规格书推导中心裁剪驱动，在本板完成
**640×360 RAW、30分钟平均359.832fps、647,698帧无序号异常**的采集核验。
实现、构建命令、失败对照及退出流程限制见 [逆向开发记录](tools/os04a10_highfps/REVERSE_ENGINEERING.md)。

2026-09-08：继续研究无裁剪模式，完成原生2688×1520、30秒平均89.958fps的RAW对照；
全视野360fps尚未实现，详见 [全视野研究记录](tools/os04a10_highfps/FULL_FOV.md)。

2026-09-08 官方版本复测：全视野1344×760@180的NV21通过30分钟；640×360@360初测出现缺号后，将ITP内部源队列从1增加到4，保持输出队列8，**NV21复测30分钟647,698帧、平均359.832fps、无序号异常**。
修复、失败对照与编码限制见 [360fps复测报告](tools/os04a10_highfps/OFFICIAL_RETEST.md)；前次录像及接口测试见 [官方功能验证报告](tools/os04a10_highfps/OFFICIAL_VALIDATION.md)。360fps连续硬件编码仍未通过。

此前纯裁剪模式覆盖原图宽、高各约24%；官方binning加裁剪模式约覆盖宽、高各48%。
驱动与测量程序位于独立目录；新增直接VIN业务入口的状态见下节。
后续需要图像/色卡验证、多轮稳定性复测、Camera API适配、重启恢复与裁剪后的相机标定，
再评估业务端消费帧率；采集359.83fps不意味着检测链路能逐帧处理。

## 全视野 180fps 业务开发模式（2026-09-08）

新增隔离 `business_capture` 入口，复用官方直接 VIN NV21 采集和 v0.2 检测/跟踪。
应用只保留最新待处理帧，NV21 全视野色度搜索后回原图局部 RGB ROI 精定位；独立线程
按 180Hz 生成预测 TargetEstimate。稳定的 `config/green_detector.conf`（480×360@60）保持原样。
新配置为 `config/green_detector_full180.conf`，只能用于此隔离入口；普通同步入口拒绝高速配置。

该入口完成一次30分钟压力复测：324245帧、180.136fps、零测量序号异常，DMA全部归还；
绿灯/装甲调用88.67/59.79Hz，TargetEstimate生成约180Hz，软件接收后源年龄P95 22.992ms。
此前长测出现291帧缺失的记录仍保留。无人工真值，目标可见时直接测量≥60Hz、曝光端到端
延迟、最坏跟踪ROI和比赛准确率尚未验收，不能将本次结果表述为比赛链路全部通过。

```bash
python3 tools/os04a10_highfps/build_official.py --out .maixpy/business-new --business
export PATH="$PWD/.maixpy/host-tools/sshpass/usr/bin:$PATH"
python3 tools/os04a10_highfps/run_device.py \
  --driver-build .maixpy/business-new --official-mode full180 --nv21 \
  --itp-depth 4 --queue-depth 4 --seconds 10 --system-media-lib \
  --remote-root /root/os04a10-tests \
  --business-config config/green_detector_full180.conf
python3 tools/os04a10_highfps/analyze_business.py RUN_DIRECTORY
```

先短测及恢复，再30秒空载（`--business-idle`）、两分钟压力（`--business-stress`），检查
通过后再做30分钟。runner临时暂停并恢复启动器，不覆盖系统库或修改自启动；原相机模式
使用已有基线程序复测。不要直接安装这个实验入口为比赛自启动应用。

新模式没有实测标定或真实模型，因此 `angles_valid=false`、`safe_for_control=false`，
NPU和Pose必须关闭。PTS保留SDK原值，曝光阶段与跨时钟偏移未知；日志明确区分接收后
软件年龄和未验证的曝光端到端延迟。逐帧日志使用固定容量异步字节缓冲；溢出或写盘失败即报错，`*.io.json`记录写入延迟和缓冲高水位。180Hz指TargetEstimate生成频率，UART/CAN尚未接通。阶段频率、失败对照、验收边界与当前结论见
[180fps业务验证报告](reports/BUSINESS_FULL180_2026-09-08.md)。公共Camera/IVPS路径仍需另行验收。

### 无电脑现场录制

独立 `dart_data_recorder` APP 已支持从 MaixCAM2 桌面打开后，在横屏调参页调整时长、曝光
和模拟增益，再点击 `START RECORDING` 录制一段 1344×760@180fps H.264，并把逐帧序号、PTS、相机参数
读回、温度和完整性结果保存到
`/root/dart_recordings/clip_NNNNNN_<label>/`。它不替换系统 sensor 库、不设开机自启，
点击 `CANCEL` 不生成录像或消耗编号；重连电脑后可逐文件校验并拉回录像。构建、安装、
现场操作和拉取说明见
[离线录制 APP](tools/offline_recorder/README.md)，本次板端安装、失败修复和完整流程实测见
[离线录制部署记录](reports/OFFLINE_RECORDER_2026-09-09.md)。
当前板端版本为 `v0.3.3`。它沿用内核文件锁，并修复了偶发向前帧序号跳变或一次已恢复
MIPI抖动导致整段录制提前退出的问题：最多2个MIPI接收错误和相应漏帧会被计数并继续
保存；错误继续增加、重复/倒退、PTS不递增或过温仍会安全失败。板端长录像使用普通调度
和深度8的VENC FIFO，避免实时采集线程挤压编码器。录制期间屏幕会主动关闭以释放显示
资源，这不是崩溃；完成后显示 `SAVED`，存在传输异常时显示带计数的警告。

### 更高检测频率实验档

阶段频率现可用`highfps.*_hz`配置；原full180仍默认90/60Hz。
新增`green_detector_full180_fast.conf`请求120/90Hz，`green_detector_full180_max.conf`
请求180/120Hz。当前场景两分钟高档实测176.14/119.33Hz，输入仍约180fps且无测量缺号，软件源年龄P95为12.26ms。
这些是检测调用频率，实验档尚未替代30分钟验证基线；详见
[调度提频报告](reports/PERFORMANCE_FULL180_RATES_2026-09-08.md)。

## 每次打开 WSL2 终端

以下环境设置只对当前终端有效，因此新终端都要执行：

```bash
source ~/maix/maixcdk-venv/bin/activate
export MAIXCDK_PATH=~/maix/MaixCDK
# USB 有线连接；若使用 Windows 网络共享则换成实际的 192.168.137.x
export MAIXCAM2_HOST=10.18.197.1
cd /home/blade_master/pnx/maixcam2_dart
```

也可以把前两条环境命令加入 `~/.bashrc`。比赛版本应固定与板端 MaixPy 对应的
MaixCDK commit，以避免 API/ABI 随升级变化：

```bash
# 板端查询版本
pip show MaixPy

# 主机按对应 MaixPy release 给出的 maixcdk_version 文件固定 commit
git -C ~/maix/MaixCDK checkout <official-commit>
```

官方入口：[MaixCDK 快速开始](https://wiki.sipeed.com/maixcdk/doc/)、
[APP 约定](https://github.com/sipeed/MaixCDK/blob/main/docs/doc_zh/convention/app.md)。

## 编译、打包和上板

首次或增删源文件后完整构建；只改已有源文件时可用 `build2`：

```bash
maixcdk build -p maixcam2
# maixcdk build2

file build/dart_green_detect
ls -lh build/dart_green_detect build/dl_lib
```

`file` 必须显示 AArch64，不能把主机 x86-64 回放程序复制到板端。生成 APP：

```bash
maixcdk release -p maixcam2
unzip -l dist/dart_green_detect_v0.2.0.zip
```

快速 SSH 调试：

```bash
ssh root@"$MAIXCAM2_HOST" 'mkdir -p /root/dart_green_detect'
scp build/dart_green_detect config/green_detector.conf main.sh \
    root@"$MAIXCAM2_HOST":/root/dart_green_detect/
scp -r build/dl_lib root@"$MAIXCAM2_HOST":/root/dart_green_detect/

ssh root@"$MAIXCAM2_HOST"
cd /root/dart_green_detect
chmod +x dart_green_detect main.sh
./main.sh
```

`main.sh` 会同时加入应用 `dl_lib`、`/opt/lib` 和系统动态库路径，避免
`libax_sys.so` 找不到，并补齐 MaixCDK `nn` 依赖的 ALSA SONAME 别名。不要用会覆盖系统路径的
`LD_LIBRARY_PATH=./dl_lib ./dart_green_detect`。

正式安装：

```bash
scp dist/dart_green_detect_v0.2.0.zip root@"$MAIXCAM2_HOST":/root/
ssh root@"$MAIXCAM2_HOST" \
  '/maixapp/apps/app_store/app_store install /root/dart_green_detect_v0.2.0.zip'
ssh root@"$MAIXCAM2_HOST" \
  'cd /maixapp/apps/dart_green_detect && sh ./main.sh'
```

若 WSL2 mirrored 模式下 PowerShell 能连而 Linux `ssh` 停在 `O_NONBLOCK`，可直接在
WSL 终端调用 Windows OpenSSH：

```bash
/mnt/c/Windows/System32/OpenSSH/ssh.exe root@"$MAIXCAM2_HOST"

PKG_WIN=$(wslpath -w "$(realpath dist/dart_green_detect_v0.2.0.zip)")
/mnt/c/Windows/System32/OpenSSH/scp.exe "$PKG_WIN" \
    root@"$MAIXCAM2_HOST":/root/
```

查询板端相机 Sensor：

```bash
python3 -c 'from maix import camera; print(camera.get_device_name())'
```

## 相机与几何标定

[默认配置](config/green_detector.conf) 是安全工程起点，不是比赛标定结果。

1. 固定镜头、焦距、安装姿态和 480×360 模式，使用棋盘格重新求
   `fx/fy/cx/cy/k1/k2/p1/p2/k3`，不能缩放沿用 1280×720 参数。
2. 关闭自动曝光和自动白平衡。依次测试 `100/200/400/800/1600 μs` 及增益，选择
   “运动拖影不超过 1.5 px 且 25 m 灯仍可见”的最长曝光；500 μs 只是起点。
3. 实测当前赛季两灯条中心距、灯条长度、绿灯相对装甲中心距离，填入
   `target_geometry.*` 后再启用位姿。
4. 红蓝比赛配置必须分别保存，并显式设置 `armor.expected_color`。

更换镜头、分辨率、曝光、目标实物或安装方向后都要重新标定。

## 主机测试和九段视频回放

识别帧率与耗时以 **MaixCAM2 板端实测**为准。主机回放用于检查算法和生成可视化，
其 FPS 不代表设备性能。对已有录像使用[板端回放基准](tools/video_replay/README.md)，
分别记录解码、检测、视觉运动和完整回放耗时；无摄像头时可使用
[NV21 录像测速入口](tools/video_replay/NV21_BENCHMARK.md)，预解码短窗口并按 180 Hz
送入同一 `HighFpsPipeline`。相机恢复后用上述 `business_capture` 入口测量真实采集频率、
直接检测频率及预测输出频率。
预测输出的 180 Hz 不能写成识别达到 180 FPS，录像回放也不能代替实时采集验收。

NV21 流程会保留尚在确认的候选 ROI，并在短暂漏检时继续查看有效预测区域；
候选确认有次数和时间上限，失败后按空间访问记录继续搜索其他候选。
尚未确认时，实际检测失败会清空命中历史，需要重新获得连续的实际命中；
计划跳帧不清空也不增加命中，已确认目标仍保留原来的短时遮挡预测。
候选阶段只搜索完整绿灯及周围背景，绿灯确认后再扩大到板子范围并扫描双灯条，
避免近景候选尚未确认就被大 ROI 拖慢。未确认的绿灯不会累计灯条确认次数。
双灯条默认要求 3 次几何一致的新扫描（`armor.confirmation_hits=3`），
相邻扫描最大间隔为 100 ms（`armor.confirmation_max_gap_ms=100`）；
缓存输出不增加确认次数，被图像边界截断的灯条不作为完整形状参与配对。
每条灯条还必须有超过其余两个通道的颜色支持，并在垂直于条带的方向形成两侧下降的
亮度分布。检查允许白亮灯芯偏在红色光晕一侧，但要求亮芯与色彩支持相连，且沿灯条
多个位置一致，避免借用邻近背景高光。明亮背景中的深饱和红/蓝灯仍可能因亮度差不足漏检。
这套规则不使用录像帧号、固定目标位置或 YOLO。验证时需同时检查无目标误检、
真实观测连续性和板端耗时，不能把更稳定的错误目标当作改进。

快速归一化路径默认开启灯斑外观验证（`appearance.enabled=true`）。每次新观测都必须
满足明确绿色支持、相对背景的亮度差、完整连通区域和紧致形状；弱绿色光晕用于恢复形状，
还需至少两个较强绿色像素支持颜色判断，其绿色信号须达到完整组件峰值的一半，避免暗色
边缘给亮灰色反光提供身份。白灯芯以局部峰值为参考，避免把灰色支架连入灯斑。
组件延伸到验证窗口外、细长结构或有明显矩形角部的大色面会被拒绝。观测位置改用完整组件的
亮度加权中心，最终输出仍经过原跟踪器滤波；历史关联和较低的跟踪分数门槛不能绕过这些外观条件。
此外检查相对背景 40% 和 50% 亮度截面的实体填充程度，以及可分辨亮芯的轴比，
减少弱光晕连接多条笔画后被误认为灯的情况。80% 亮芯层采用组件亮度 P95 作为峰值参考，
容忍少量高光与灯面纹理；少于 8 个支持像素时不强套该层的连续形状假设。

小尺度候选仍使用既有配置，较大尺度按倍频扩展，验证半径上限为 128 px；粗排与精检使用
一致的亮度贡献，并同时保留绿色峰和亮核种子。NV21 全场搜索采用严格绿色差值和周边对比，
每 32×32 区域最多一个候选进入多尺度复查，最后保持原有前五候选预算。
源 ROI 适配器保留检测计数，使跟踪窗口及定期全域刷新按配置生效；跟踪窗口为完整灯斑
保留周围背景。精检缓存按实际候选窗口扩展，减少重复扫描，灯条验证复用相同采样。
精检前对相同位置和直径的假设去重，保留首次出现顺序，仍最多精检 128 个假设，
避免重复小尺度候选挤掉完整灯体候选。

可配置的通用外观参数为 `appearance.min_green_margin=0.08`、
`appearance.min_color_fraction=0.12`、`appearance.min_relative_contrast=0.12`、
`appearance.max_axis_ratio=2.5`、`appearance.min_core_fill=0.80`。
这些字段有默认值，原配置文件无需修改。
颜色证据不足、被遮挡或超出可验证范围的目标可能暂时不确认；短时预测仍单独标记。
仅凭外观不能保证排除所有相似绿色灯具或物体，验收应使用独立正负录像，同时记录
误锁持续时间、漏检、定位误差和板端检测调用帧率。
基础实现见 [外观验证与稳定性报告](reports/APPEARANCE_STABILITY_2026-09-20.md)，
后续背景标记误锁修复及复验见 [31 秒误识别修复报告](reports/CLIP31_FALSE_POSITIVE_FIX_2026-09-20.md)。

```bash
cmake -S tests -B build/tests -DCMAKE_BUILD_TYPE=Release
cmake --build build/tests -j
ctest --test-dir build/tests --output-on-failure

sudo apt install -y libopencv-dev ffmpeg
cmake -S tools/video_replay -B build/video-replay -DCMAKE_BUILD_TYPE=Release
cmake --build build/video-replay -j
```

单段回放默认先转换到部署分辨率 480×360；只有分析原分辨率时才加
`--native-resolution`：

```bash
build/video-replay/dart_video_replay \
  --input recordings/maixcam2/2026-08-30_usb/2.mp4 \
  --output /tmp/2_detected_v0.2.mp4 \
  --jsonl /tmp/2_detected_v0.2.jsonl \
  --config config/green_detector.conf
```

九段全量回归：

```bash
python3 tools/run_v02_regression.py \
  --input-dir recordings/maixcam2/2026-08-30_usb \
  --output-dir recordings/maixcam2/2026-08-30_usb/results_v0.2
```

输出包括九个叠加可视化视频、逐帧 JSONL、`replay_summaries.json`、
`report_v0.2.json` 和 `REPORT_v0.2.md`。未标注录像的有效输出率不是准确率；使用
[区间标注示例](config/regression_annotations.example.json) 后，报告才会计算目标出现后的
捕获延迟和无目标区间控制级误报。候选召回、关键点误差、视线角误差和错误换轨还需要
[逐帧真值示例](config/frame_ground_truth.example.jsonl)：

```bash
python3 tools/evaluate_replay.py RESULT.jsonl \
  --ground-truth-jsonl frame_ground_truth.jsonl \
  --output-json reports/generated/replay.json \
  --output-markdown reports/generated/replay.md --enforce
```

## 数据集与关键点模型

数据要求、目录结构、关键点顺序和 Pulsar2 步骤见
[模型说明](models/README.md)。关键工具：

```bash
# 从回放候选导出 8×直径 ROI，供标注或硬负挖掘
python3 tools/dataset/extract_candidate_rois.py \
  --video VIDEO.mp4 --jsonl RESULT.jsonl --output DATASET_ROIS

# 完整录像分组，第三场地锁定为最终测试，防止相邻帧泄漏
python3 tools/dataset/split_by_sequence.py \
  --manifest DATASET_ROIS/manifest.jsonl --output SPLITS \
  --test-venue venue_c

python3 tools/dataset/validate_pose_dataset.py --root POSE_DATASET

# 选 100 张覆盖距离/颜色/模糊/场地的 INT8 代表图
python3 tools/dataset/select_calibration_images.py \
  --manifest DATASET_ROIS/manifest.jsonl --dataset-root DATASET_ROIS \
  --output CALIBRATION_100 --count 100
```

训练/导出为固定输入 ONNX 后，必须按官方 MaixCAM2 手动 Pulsar2 流程生成
NPU2/NPU1 两个 `.axmodel` 和 `.mud`；在线转换目前只覆盖 Detect。转换后执行：

```bash
python3 tools/model/stage_runtime_model.py --mud /path/to/dart_target_pose.mud
```

模型会进入 `models/runtime/`，发布时安装为应用的 `models/`。确认实机 P95 后再把配置改为：

```ini
npu.enabled=true
npu.required=true
npu.model_path=models/dart_target_pose.mud
npu.input_size=256
```

## JSON v2 和控制语义

程序内部每帧生成 `TargetEstimate`。终端默认每 6 帧输出一行 JSON（60 FPS 时约 10 Hz）；
v2 包含：时间戳、测量年龄、五级状态、是否预测、传统检测是否运行/耗时、
`safe_for_control`、绿灯、双灯条/装甲中心、可选位姿、瞄准点、相机系单位视线、
yaw/pitch、角速度、协方差和模型耗时；仅在 `debug.log_candidates=true` 时附候选列表，
同时保留 v0.1 的平铺绿灯字段。
统一 C++ 检测入口为 `process(frame, timestamp_us, MotionPrior*)`；第三个参数可传
`nullptr`，后续接入 ICM-42688 时无需更改检测/控制数据结构。

控制端必须以 `safe_for_control` 为最终门，不能只看 `valid`：预测超过 2 帧/35 ms、
颜色为 auto、NPU 已启用但模型不可用/不一致等情况都会报告状态但禁止控制。

板端日志还包含 `processing_ms`、`capture_interval_us` 和 `rss_kb`。短时逐帧性能测量可临时设置：

```ini
debug.json_log_every_n_frames=1
debug.log_candidates=false
```

应重定向到板端文件，避免 SSH 终端渲染影响循环。完成 30 分钟运行后：

```bash
python3 tools/evaluate_device_log.py detections.jsonl \
  --output reports/generated/device_30min.json
```

脚本检查持续时间、端到端 P95、NPU P95、估算丢帧率和 RSS 增长。温度/降频仍需同时
在板端系统日志中观察。

## 验收原则

离线门槛为正候选召回 ≥99.5%、关键点原图 P95 ≤2 px，并在锁定硬负测试集保持
控制级误报为 0。当前实机要求 480×360@60fps、端到端 P95 ≤30 ms、模型 P95 ≤20 ms、
丢帧率 <0.1%、30 分钟无持续内存增长。最终还需至少 300 次独立 15–25 m 试验，覆盖
±5°、红蓝、全滚转和等效运动；指标详见实施状态文档。

“接近 100%”只对已定义的距离、速度、光照、捕获锥和测试分布成立。低置信度结果必须
失效保护，不能为追求表面连续率而伪装成有效控制量。

## Git 与大文件

Git 保留源代码、配置模板、测试代码、工具使用说明、设计文档和模型契约。
`reports/` 下的测试报告、`recordings/` 下的全部采集内容、`artifacts/` 下的测试摘要、
图表及原始结果均仅在本地保存；`artifacts/README.md` 是唯一的产物目录例外。
下载到 `docs/` 的 PDF 规格书、训练数据、`.pt/.onnx/.axmodel`、构建目录、发布包和本地
SDK 也默认忽略。新产生的测试输出应放入上述忽略目录，避免散落在源码目录中。

已跟踪的报告和产物需要取消 Git 索引跟踪，文件仍保留在本地；`.gitignore` 不会清除
历史提交中的内容。完整材料应通过单独的备份或制品存储交接，规则见
[验证产物的版本管理](artifacts/README.md)。

`.clangd` 含本机 SDK/工具链绝对路径，因此保持忽略；新电脑先生成
`build/compile_commands.json`，再执行 `cp .clangd.example .clangd`，必要时添加该电脑的
MaixCDK include 路径。
