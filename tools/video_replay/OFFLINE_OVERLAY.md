# 连续原视频识别叠加

`board_nv21_offline.cpp` 用于在 MaixCAM2 上完整处理原录像，生成逐源帧识别结果，随后在主机渲染单画面叠加视频。它复用同一套 NV21 色彩转换、全场候选、ROI 调度、绿灯/双条检测及运动估计组件；不启动相机，不引入模型，不调整检测参数。

这是连续离线诊断工具。跟踪和检测调度采用原视频时间轴，不受软件解码耗时影响；不模拟实时线程竞争或丢帧。运动结果延迟到下一次观测消费。其运行耗时不能替代 `board_nv21_replay` 的实时缓存回放 FPS。

构建独立板端程序：

```bash
python3 tools/video_replay/build_board_benchmark.py \
  --pipeline offline --out .maixpy/offline-overlay-build
```

把二进制、`dl_lib` 和配置复制到板端独立目录，在新的输出目录执行：

```bash
/path/to/board_nv21_offline \
  --input /path/to/record_180fps.mp4 \
  --config /path/to/green_detector_full180.conf
```

输入须为 1344×760、标称 180 fps、连续 PTS 的录像。默认处理到 EOF；`--max-frames` 仅用于有界检查，截断结果不会声明覆盖整段。输出 `observations.jsonl` 和 `offline_summary.json`，拒绝覆盖原结果。

`observations.jsonl` 按 90 Hz 调度记录每次真正检测的源帧，保留原始 PTS、源帧号、ROI、候选/直接/预测状态及双条来源时间戳。视频中未安排检测的输入帧不会消耗漏检预算。原始数据保持 180 fps 解码，不能在推理前先减到导出帧率。

主机生成单画面视频：

```bash
python3 tools/video_replay/render_source_overlay.py \
  --source exact_board_input.mp4 \
  --observations results/observations.jsonl \
  --summary results/offline_summary.json \
  --run results/run.json \
  --output source_detection_overlay.mp4
```

`run.json` 为运行来源记录，须包含真实 `exit_code`、`input_sha256`、`board_machine`（板端为 `aarch64`），以及 `result_sha256` 中两个结果文件的真实 SHA；本次实际生成方式保存在 `artifacts/clip13_source_overlay_20260920/run_on_board.py`。渲染器要求完整源片运行成功，并检查输入 SHA 与原帧数。主机 smoke 会明确标为非板端，不能冒充板端检测。

输出保持原分辨率和播放速度，以 30 fps 导出，逐帧只使用自身源帧的检测结果，不插值、不从相邻帧补框。绿色表示新绿灯观测，黄色是未确认候选，橙色虚线是预测；红色仅绘制时间戳匹配的本帧双条新扫描，缓存双条不画。输出同名 JSON 保存帧关联、绘制坐标及 SHA，导出时长最多多出不足一个输出帧周期。
