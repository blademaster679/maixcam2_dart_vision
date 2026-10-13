# 180 Hz 视频输入下的板端 RGB＋NPU 测速

本工具链接已编译 MaixCDK 应用的 `main/libmain.a` 和真实 Maix 图像/NPU库，
调用 `GreenLightDetector::process()`、`VisualMotionEstimator` 及
`create_yolo_pose_validator()`。适用于当前已部署的 480×360 RGB、320×320
中心点模型配置。它不代表尚未接入 NPU 的原生 1344×760 NV21 高帧率通路。

构建前先完成正常应用构建，保持源文件与应用静态库一致：

```bash
python3 tools/model/build_video_model_fps.py --out artifacts/video_fps_build_new
```

将独立可执行文件上传到板端测试目录，使用部署应用已有的动态库和模型。
示例（工作目录是现有应用目录，输出必须是新文件）：

```bash
LD_LIBRARY_PATH=./dl_lib:/opt/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} \
  /root/green_center_20261007/video_fps_v2/board_video_model_fps \
  /root/dart_clip13_benchmark_20260920/clip13.mp4 \
  green_detector.conf 26 20 33 /tmp/video_fps_new.csv > /tmp/video_fps_new.log 2>&1
```

位置参数依次为视频、配置、视频起点秒数、测试秒数、NPU调用间隔毫秒、CSV路径。
间隔33使用当前调度；1仅解除间隔限制，仍保留传统检测/模型交替和候选门控；
-1关闭模型供对照。它们不是“每一帧都运行模型”的模式。

测量规则：

- 确认源视频名义180 FPS且连续180帧的PTS间隔满足180Hz。
- 先解码并按色彩元数据转换/缩放到480×360 RGB，再开始计时。
- 缓存180帧（1秒，约89MiB），循环回放；循环边界是人为时序跳变。
- 按单调时钟产生180Hz虚拟输入序号；处理开始时只取最新帧，跳过过期帧。
  这模拟最新帧输入，**不是实测摄像头180FPS，也不包含生产者线程调度开销**。
- 实际运行传统检测、装甲检测、运动补偿、跟踪融合和候选模型验证。
- 包装真实 `validate()` 计数，模型时间含裁剪、letterbox、推理及后处理。
  与 `target.model_ran` 交叉检查。预测、缓存、无候选帧不计模型调用。
- CSV在计时结束后写出；不包含显示、编码、JSON输出及摄像头/ISP的资源竞争。
- 统计统一采用完成时间窗口[3秒, 测试时长−0.5秒)，剔除启动和尾部。
  阶段耗时用于诊断；帧率用实际调用次数÷稳态墙钟时间，不能直接用1000/耗时替代。
- 当前应用仍是传统检测与NPU交替，`processed_fps`含预测更新；
  `classical_fps`为传统检测次数，`model_fps`才是实际模型验证次数。
  `green_observed_fps`以及模型阳性数不能作为识别准确率。

下载同名CSV和log后独立核算：

```bash
python3 tools/model/analyze_video_model_fps.py artifacts/video_model_fps_20261007
```

该分析会检查序号单调、缓存索引映射，并从CSV重新计算所有帧率，与板端摘要核对。
真实相机端到端性能需在接摄像头后另测；短窗口循环的结果也不能直接外推到整段录像。

## 导出带真实识别结果的录像

新版CSV同时记录当帧绿灯观测/预测、实际模型ROI、模型中心及置信度。
仅在真实模型调用的帧记录模型坐标，不把历史模型坐标当成新推理。
先用新版程序重新运行四个起点（26、29.5、62、74秒），保存同名CSV/log，再执行：

```bash
python3 tools/model/render_video_model_fps.py \
  --video artifacts/video_model_overlay_20261007/board_source.mp4 \
  --results artifacts/video_model_overlay_20261007 \
  --output artifacts/video_model_overlay_20261007/recognition_v4_with_fps.mp4
```

源视频必须与板端使用的文件一致；不能仅凭录像名称推定相同。
视频以30 FPS编码，按测试墙钟完成时间显示最后完成的结果帧；阻塞期间保持旧帧。
粉色十字仅表示该帧模型中心点，青色框为模型验证区域，绿色为传统观测，黄色为预测。
右侧保留最近一次模型ROI原帧并显示其年龄，避免将旧模型坐标叠到新画面。
平均帧率来自对应重测日志；滚动调用数来自截至当前播放时间的最近1秒完成记录。
此文件仍是四个1秒缓存窗口各循环20秒的测试展示，不是整段连续录像评估。

## 完整原片逐帧识别与原画幅叠加

`board_video_model_full.cpp` 复用相同FFmpeg色彩转换、真实Maix图像、C++传统检测及V4模型接口，
连续解码并处理源视频全部帧。跟踪时间来自原始PTS，保持原模型33ms调度与传统/模型交替。
该离线任务不丢弃处理超时帧，因此耗时较长；它的吞吐不代表180Hz实时输入的表现。

```bash
python3 tools/model/build_video_model_fps.py \
  --source tools/model/board_video_model_full.cpp --out artifacts/full_video_build_new
# 板端，在现有application工作目录，使用现有模型和动态库：
/path/to/board_video_model_full /path/to/clip13.mp4 green_detector.conf /new/observations.jsonl
```

运行成功后生成 `observations.jsonl.summary.json`，记录完整源帧数、PTS和耗时。
每行保存来源帧号、该帧完整融合结果、候选列表，以及仅属于该帧的模型ROI、中心、分数。
模型几何不从其他帧移贴；装甲双条只在该帧真实扫描且确认通过时绘制。

下载日志并校验板端退出码后，保存 `exit_code` 为真实退出码，以及 `provenance.json`
中的 `source_sha256` 和 `observations_sha256`，然后运行：

```bash
python3 tools/model/render_video_model_full.py \
  --source artifacts/video_model_overlay_20261007/board_source.mp4 \
  --results artifacts/video_model_full_20261007 \
  --output artifacts/video_model_full_20261007/recognition_full_original_v4_180fps.mp4
```

输出保留原1344×760画幅、全部源帧、180FPS、原时长和音轨（若存在），没有剪段循环，
也不将480×360识别输入替代原画面。识别坐标分别按源宽/480、源高/360映射回原片。
界面“识别计算吞吐”和“模型调用吞吐”为最近180源帧的实际识别计时统计，不含解码；
“最近模型”状态标明年龄，只作为文字提示，不把过期中心坐标画到新帧。
输出旁的JSON审计文件记录源/结果/视频哈希、源与输出帧数、坐标比例和绘制计数。
