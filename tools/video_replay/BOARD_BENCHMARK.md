# 核心板离线识别测速

`board_replay_benchmark` 在板上读取录制 MP4，用 FFmpeg 软件解码、转换 RGB，运行
与主机回放一致的 OpenCV Image 适配层、绿灯/双灯条检测和视觉运动估计。
无需摄像头，不调用 VIN、传感器或显示接口；不覆盖系统库。
这项测试验证板端 CPU 执行 RGB 算法的速度，不能代表硬解或实时 NV21 ROI
生产管线的吞吐。录像经过有损压缩，RGB 与原始 VIN 像素也不完全相同。

从本地缓存的 MaixCAM2 SDK 交叉编译：

```bash
python3 tools/video_replay/build_board_benchmark.py \
  --out .maixpy/clip13-board-replay
```

输出目录存在时构建器拒绝覆盖。部署只需独立目录中的 `board_replay_benchmark`、
`dl_lib/`、`build_manifest.json`，以及待测配置和视频；对象文件及日志不必传输。
确认不存在 `BUILD_INCOMPLETE` 后使用。`dl_lib` 包含匹配的 FFmpeg
avformat/avcodec/avutil/swscale/swresample 和 OpenCV core/imgproc。
运行前检查 `ldd ./board_replay_benchmark` 全部解析；C++/OpenMP/GCC 运行库
仍由板端系统提供。

先测 300 源帧：

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
./board_replay_benchmark \
  --input clip13.mp4 --config green_detector_full180_clip13.conf \
  --native-resolution --max-frames 300 --warmup-frames 30 \
  --decode-threads 1 \
  --jsonl smoke.jsonl --summary smoke.json --first-rgb first.rgb
```

移除 `--max-frames 300` 即处理全片。默认 `--frame-step 1` 不抽帧；
`--frame-step 6` 在 180 fps 原片上处理 0、6、12……帧，仅用于明确标记的抽样测试。
始终不生成叠加视频；不传 `--jsonl` 可测无逐帧输出的版本。两配置比较应使用相同
二进制、视频、抽样间隔、预热长度、线程数、JSONL 设置，并记录温度、频率及其他
进程负载。构建清单含二进制、源码和依赖 SHA256；运行记录还应保存视频和配置 SHA256。

## 计时口径

- 所有时长来自 `steady_clock`，没有按视频时间睡眠。
- 预热是连续前 N 个**处理帧**，正常推进跟踪状态，也写入 JSONL（`warmup=true`）；
  只从测速统计扣除，不重置跟踪器、不倒退时间。
- JSONL `frame` 是原视频零起始索引，`timestamp_us` 为源帧号除以标称 FPS；
  与主机回放时间口径一致，不能恢复录制丢帧造成的真实时间间隙。
- `wall_processed_fps` 是扣除预热后的处理帧数/实际总耗时，包含解码、RGB 转换、
  检测、运动估计、逐帧日志序列化/写入及最终输出刷新。
- `wall_source_fps` 使用读取源帧数；抽样时与识别 FPS 含义不同。
- `detector_only_fps` 只由 `GreenLightDetector::process` 总时间计算；
  `detector_and_motion_fps` 另含视觉运动估计；均不含视频解码，不能当作端到端吞吐。
- `read_demux` 计 `av_read_frame`（解复用和读取，无法区分页缓存与物理读盘）；
  `decode` 计 FFmpeg 发送/接收接口；`decode_demux` 是整个下一帧调用。
  抽样跳过的源帧仍需解码，这三个阶段包含这些帧。
- `rgb_convert_resize`、`detector`、`visual_motion`、`detector_and_motion`、
  `json_io` 按实际处理帧统计，给出样本数、总时长、均值、p50、p95、最大值。
  视觉运动未运行时只计条件检查开销，其统计按所有处理帧取样。
- `json_io` 含序列化和流写入；最终刷新另列 `final_json_flush_ms`。
  只执行用户态 `flush/close`，没有 `fsync`，不是断电持久化性能测试。
- `armor_frames` 含有效缓存结果；`armor_detection_calls` 是实际调用次数，
  `direct_armor_frames` 是当帧执行检测且有效的次数。
- 发现解码错误、损坏/掩盖错误帧会失败；正常 EOF 时，若容器提供帧数，必须与读取帧数一致。
  `--max-frames` 是读取上限，不要求到达 EOF。

## 颜色与安全范围

转换使用解码器提供的像素格式和 libswscale 默认矩阵/范围处理。第 13 段为
`yuvj420p` 全范围、BT.601 系元信息，不能人为套用 limited-range 的 Y−16 转换
当作原始 VIN 复现。`--first-rgb` 输出第一处理帧的连续 RGB888 原始字节，以便和
主机 OpenCV 解码结果比较。FFmpeg/OpenCV 版本、CPU 架构及舍入路径可能产生少量
像素差，接近颜色门槛的帧不保证两端识别完全一致。

本轮应使用 `--native-resolution`，配置分辨率也保持 1344×760。非原分辨率路径
使用 libswscale 双线性缩放，主机工具使用 OpenCV `INTER_AREA`，两者不应当作
逐像素等价。OpenCV Image 适配层的 LAB 连通域实现也不能直接视为真实 MaixCDK
Image 算子的性能；当前实验关闭 legacy LAB。

全部输出强制 `safe_for_control=false`、`angles_valid=false`、`pose.valid=false`。
NPU 启用配置会被拒绝，因为本工具没有 NPU validator。输出仅用于离线诊断。

## 主机接口回归

以下只验证接口、时间和计数正确性，主机时间不能作为板端性能结果：

```bash
g++ -std=c++17 -O2 -pthread -Itools/video_replay -Imain/include \
  $(pkg-config --cflags opencv4 libavformat libavcodec libavutil libswscale) \
  tools/video_replay/board_replay_benchmark.cpp tools/video_replay/maix_image_opencv.cpp \
  main/src/config.cpp main/src/green_detector.cpp main/src/green_detector_core.cpp \
  main/src/target_fusion.cpp main/src/target_json.cpp main/src/visual_motion.cpp \
  $(pkg-config --libs opencv4 libavformat libavcodec libavutil libswscale) \
  -o /tmp/dart_board_replay_host
python3 tools/video_replay/test_board_benchmark.py --binary /tmp/dart_board_replay_host -v
```

回归生成独立的合成视频，验证源帧索引、预热计数、预热不改变跟踪结果、安全失效、
完整 EOF、RGB 导出、NPU 拒绝及截断视频不能报告完整成功。
