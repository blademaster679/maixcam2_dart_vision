# 板端 NV21 业务管线录像测速

`board_nv21_replay` 直接调用 `main/src/nv21_pipeline.cpp` 的 `HighFpsPipeline`：
同一套 NV21 全幅候选搜索、ROI 选择与 RGB 转换、绿灯跟踪、双条检测、异步运动估计、
最新帧覆盖策略及 180 Hz 预测输出。不初始化摄像头、显示或传感器。

## 构建与运行

```bash
python3 tools/video_replay/build_board_benchmark.py \
  --pipeline nv21 --out .maixpy/clip13-nv21-board
```

复制输出目录中的 `board_nv21_replay` 和 `dl_lib/` 到板端独立目录，并复制配置。
每轮在新工作目录运行，保留原录像；工具拒绝覆盖自己的既有日志。

```bash
mkdir small_optimized
cd small_optimized
../board_nv21_replay \
  --input /root/dart_clip13_benchmark_20260920/clip13.mp4 \
  --config ../optimized.conf \
  --start-seconds 26 --cache-frames 90 --seconds 20 --summary replay.json
```

拉回整个工作目录后分析：

```bash
python3 tools/video_replay/analyze_nv21_replay.py PATH_TO_RUN
```

## 测量方式

先将指定视频窗口软件解码为内存 NV21 帧，再在独立计时阶段按 180 Hz 循环提交。
默认 90 帧约 131.5 MiB，代表原视频的半秒；缓存有内存上限，测试有时间上限。
解码期间不运行识别管线，其耗时单独记录。这样软件解码能力不会限制测速阶段的输入率。

每次提交使用新帧对象，共享只读像素缓存；生命周期由 `shared_ptr` 维护。
实际接收时间使用主机单调时钟，序号和合成 PTS 跨循环递增，源视频 PTS 另存。
晚于输入调度的过期 tick 被跳过，不连续补发旧帧。`feed.csv` 保存源帧映射与循环号，
`replay.json` 保存缓存、色彩处理、时钟及喂入统计，`business.json` 保存管线累计计数。
分析默认排除首 3 秒和末 0.5 秒，用同一稳态时间窗口计算各阶段实际调用频率。

录像的色彩范围和矩阵须按元数据解释，然后转为现有 `nv21_rgb_region()` 所期望的
BT.601 limited-range NV21。此归一化不代表已经验证了传感器原始 VIN 的颜色语义；
有损录像、4:2:0 色度和舍入可能影响微小灯点的阈值判断。

## 如何解读

- `feed` Hz：实际向业务管线提交的频率，不是相机实测采集 FPS。
- `vision` / `green` Hz：实际精检测调用频率；配置的 90 Hz 只是调度目标。
- 直接绿灯观测 Hz、双条调用/直接有效观测 Hz：区分确认、预测及缓存，不等于准确率。
- `detect_us`：ROI 内检测、跟踪和融合；它不包含候选搜索、转换、运动线程或排队。
  `1000/平均detect_ms` 是单阶段计算吞吐，不能当作整条管线 FPS。
- ROI 面积、搜索候选数、绿灯状态、双条调用量用于解释不同窗口的实际负载。
  `analysis.json` 的 `roi_stage_ms` 按 ROI 尺寸分别统计检测和视觉总耗时，
  避免大量小 ROI 调用掩盖少量大 ROI 的开销。
- `slot_replaced` 是等待槽被更新帧替换，`scheduled_skipped` 是调度主动跳过，
  `shutdown_discarded` 是停止时丢弃；它们与输入线程错过 tick 的含义不同。
- 180 Hz 输出主要是预测与状态更新，不是 180 次直接识别。

这是重复短窗口的板端业务处理能力测试，循环边界有人工跳变，不能据此宣称整段
76 秒视频均可实时处理或识别准确率达标。完整连续录像效果仍需另做时序与真值验证。
该入口使用现有 OpenCV RGB 图像适配器，传统像素检测源码相同；关闭 legacy LAB 分支。
它不包含 VIN DMA 映射/缓存失效/释放、ISP 竞争、相机曝光或同时录像的 VENC/写盘开销，
因此不能替代相机恢复后的完整采集验收。

`targets.jsonl` 标记 `replay_diagnostic_only=true`，接收时钟标记为
`cached_video_submit_host_monotonic`，始终不提供有效控制角度或位姿。

## 将本次板端日志导出为识别视频

```bash
python3 tools/video_replay/render_nv21_comparison.py \
  --input recordings/maixcam2/2026-09-20/clip_000013_test5min/record_180fps.mp4 \
  --results artifacts/clip13_nv21/results \
  --output artifacts/clip13_nv21/nv21_recognition_comparison.mp4
```

此工具针对本次四窗口、两配置的日志导出左右对照。需要 FFmpeg、OpenCV、Pillow 和
中文字体。按同一提交序号，将已完成的观察画到它对应的原片帧上；不是现场显示延迟的模拟。
绿灯坐标来自 `vision.csv`，双条端点必须同时匹配 `source_sequence`、
`source_received_us`、`armor_source_received_us`，不把其他帧的缓存几何当作本帧识别。

主片以 30 fps 展示 20 秒输入时间线；抽样会省略部分短暂输出，不能数视频中的框计算准确率。
末尾另给五次候选轮询、三个双条源帧和大 ROI 的停帧特写。黄色为未确认候选，绿色为
已确认直接观测，橙色为预测，青色为当前 ROI，红色为算法双条端点及连接几何。
红线不是板体分割，也不保证正确。底图是原 MP4 解码图像，不是 NV21 原始像素转储。

同名 JSON 保存章节、输入日志哈希、输出哈希和渲染口径。相邻的 `preview_*.jpg`
是静态预览。已有输出默认拒绝覆盖；重新生成时使用新的输出文件名。
