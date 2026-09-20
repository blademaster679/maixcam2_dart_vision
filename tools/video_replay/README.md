# RGB 视频离线回放

需要在 MaixCAM2 上复用实时 NV21 ROI 业务流程测速时，使用
[NV21 板端回放入口](NV21_BENCHMARK.md)；全幅 RGB 板端对照见
[RGB 板端基准](BOARD_BENCHMARK.md)。下面介绍主机 RGB 回放工具。

构建：

```bash
cmake -S tools/video_replay -B build/video-replay -DCMAKE_BUILD_TYPE=Release
cmake --build build/video-replay -j
```

默认逐帧处理、输出叠加视频及 JSONL；处理分辨率来自配置，
`--native-resolution` 使用输入视频分辨率。现有命令保持兼容。

长视频可按源视频帧号抽样，仅输出识别 JSONL：

```bash
build/video-replay/dart_video_replay \
  --input recordings/maixcam2/2026-09-20/clip_000013_test5min/record_180fps.mp4 \
  --config config/green_detector_full180.conf \
  --native-resolution --frame-step 6 --no-overlay-video \
  --jsonl artifacts/clip13/replay.jsonl
```

- `--frame-step N` 处理源帧 `0,N,2N,...`。JSONL 的 `frame` 保留源视频的
  零起始帧号，`timestamp_us=round(frame*1000000/source_fps)`。
  此时间轴来自容器标称帧率，不恢复录制时丢帧造成的真实时间间隙。
- `--max-frames N` 始终限定读取的源帧范围 `[0,N)`，不是抽样后的处理帧数。
- 保留叠加视频输出时，输出帧率为 `source_fps/frame_step`，播放速度保持不变；
  最后不足一个采样周期时，输出时长可能多出不足一个输出帧周期。
- `--no-overlay-video` 不创建视频或绘制叠加层；须提供 `--jsonl`，也可以保留
  `--output` 仅用作默认 JSONL 路径的前缀。
- 汇总的 `frames`、检测率和平均处理 FPS 均按实际处理帧计算；
  `source_frames_read` 单独给出读取的源帧数。
- 抽样会改变时序检测器接收帧的频率，确认命中数和丢失帧预算按处理帧计；
  视觉运动估计的 `interval_frames` 也按处理帧计。不同抽样间隔的结果不能直接
  当作相同采集频率下的性能。正式比较应使用同一视频、同一帧间隔及同一真值。

该工具将视频解码成 RGB，运行 `GreenLightDetector`；它不运行板端高速 NV21
ROI 转换、分阶段调度或原始帧元数据链路。逐帧 JSON 和汇总显式记录
`replay_pipeline="rgb"`、`frame_step`、`replay_timestamps="nominal_source_fps"`。
离线有效输出率不是识别准确率，也不能证明板端实时性能。

当配置请求 `camera.fps>60`（包括未经标定的 full180 配置），或关闭
`capture.enabled` 时，工具把此回放视为诊断用途，记录
`replay_diagnostic_only=true`，强制 `angles_valid=false`、
`safe_for_control=false`、`pose.valid=false`，清空视线角及相关角度量。
关闭捕获锥本身不证明相机未标定，这是此回放工具采用的保守诊断约定。
像素坐标、绿灯和双灯条识别结果仍正常输出，可用于标注比对。

独立 CLI 回归（需要 Python 的 OpenCV 和 NumPy）：

```bash
python3 tools/video_replay/test_video_replay.py \
  --binary build/video-replay/dart_video_replay
```
