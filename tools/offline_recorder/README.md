# MaixCAM2 离线全视野录制 APP

`dart_data_recorder` 用于电脑不在现场时，从 MaixCAM2 桌面录制 OS04A10
1344×760@180 fps 数据。打开应用后先显示横屏调参页，可直接调整录制时长、曝光和模拟
增益；只有再次按下并松开绿色 `START RECORDING` 才会录制，按 `CANCEL` 直接返回且不
占用录像编号。输出保存在
`/root/dart_recordings/clip_NNNNNN_unlabeled/`。

每段目录包含 `record.h264`、采集与编码逐帧 CSV、相机设置读回、温度/MIPI状态、
寄存器和完整性结果。只有通过全部检查的目录才包含 `.complete`；失败目录以
`.failed` 结尾。板端时钟可能不准确，因此唯一标识使用持久递增编号，不使用日期。

默认配置见 `recorder.conf`。如需覆盖，在板端创建 `/root/dart_data_recorder.conf`，
支持同样的 `key=value` 项。`session_label` 只能包含英文字母、数字、下划线和连字符。

构建经过修改的官方录制程序和 APP 包：

```bash
python3 tools/os04a10_highfps/build_official.py \
  --out .maixpy/official-offline-recorder
python3 tools/offline_recorder/build_package.py \
  --driver-build .maixpy/official-offline-recorder
```

安装包为 `dist/dart_data_recorder_v0.4.0.zip`。它包含进程内使用的官方 OS04A10
sensor 库副本和当前 MaixCDK 链接所需的应用运行库，不覆盖 `/opt/lib`，也不设置开机自启。

`v0.3.1` 使用内核 `flock` 防止重复启动。锁在正常退出、强制终止和设备重启后都会自动
释放；升级时还会回收 `v0.3.0` 遗留的空目录锁，不再因残留
`/root/dart_recordings/.recording.lock` 反复报告 `Recorder is already active`。

`v0.3.2` 不再因为一次向前帧序号跳变而中途退出。录制器会继续保存全部可用帧，在
`capture.json` 中记录 `sequence_gap_events` 和 `sequence_missing`；重复、倒退或PTS不递增
仍作为致命时序错误。采集主线程使用SCHED_FIFO优先级10降低调度导致的漏帧概率。录制
期间显示关闭以避免VO与180fps采集争用，开始前会明确提示，结束状态会保持显示；若存在
漏帧则显示 `SAVED WITH GAPS`，这类录像可用于训练，但不能作为无丢帧性能验收证据。
因此按下开始后暂时黑屏是预期行为；只有超过所选时长后仍未返回 `SAVED`、
`SAVED WITH GAPS` 或明确的 `ERROR`，才应视为异常并检查桌面运行日志。

`v0.3.3` 针对现场真实出现的一次MIPI自动恢复增加有界容错：默认允许接收器累计2个错误，
并将 `mipi_errors_max`、温度上限和具体健康失败原因写入 `capture.json`。这类录像会显示并
拉取为警告，不能作为无丢帧验收数据；累计错误超过2仍停止，并提示检查相机FPC连接。
默认值可在板端覆盖配置中用 `max_mipi_errors=0|2` 选择严格或有界模式，不接受更宽松值。
同时取消离线录像的 `SCHED_FIFO`，把硬件VENC输入/输出FIFO固定为8；两次连续30秒板端
对照共编码10809帧，均约180.135fps且无缺帧、MIPI或编码错误。

安装到当前有线连接的 MaixCAM2：

```bash
export PATH="$PWD/.maixpy/host-tools/sshpass/usr/bin:$PATH"
python3 tools/offline_recorder/install_on_device.py
```

现场使用：先打开系统相机确认构图，退出相机后点击“飞镖数据录制”；在调参页用每行的
`-` 和 `+` 选择参数，按下并松开绿色 `START RECORDING`。屏幕显示红色 `RECORDING`
时不要再次启动其他相机应用；看到绿色 `SAVED` 后即完成。若不想录制，按 `CANCEL`，
不会创建目录或递增编号。建议纸面记录 clip 编号、距离、颜色、滚转、运动方式和光照。

| 屏幕参数 | 可选档位 | 说明 |
| --- | --- | --- |
| `TIME` | 3/5/10/15/20/30秒，以及1/2/5/10分钟 | 单段录像时长 |
| `EXPOSURE` | 100/200/400/500/800/1200/1600/2400/3200/4800μs | 均小于180fps帧周期；用于运动模糊与远距离亮度权衡 |
| `GAIN` | 1/2/4/8/16倍 | OS04A10模拟增益，界面值会转换为ISP的U22.10原始值 |

按 `DEFAULT` 恢复 `recorder.conf` 或板端覆盖配置的启动值。按 `START RECORDING` 后，本次
参数会写入录像的 `record_context.json`，并保存到
`/root/dart_data_recorder_ui_state.conf`，下次打开继续沿用；`CANCEL` 不保存刚才的修改。

`v0.2.1` 起已兼容 MaixAPP 桌面启动器附带的空字符串参数；旧版点击后若显示
`Unknown option:`，应安装当前版本，无需修改相机或系统 sensor 库。

重新连接电脑后拉取全部已完成录像；默认保留板端副本：

```bash
export PATH="$PWD/.maixpy/host-tools/sshpass/usr/bin:$PATH"
python3 tools/offline_recorder/pull_recordings.py
```

拉取时会逐文件比较板端和本地 SHA-256；全部一致后，本地每段增加
`pull_manifest.json`。确认本地文件和备份无误后，再单独清理板端旧录像；拉取工具本身
不会删除数据。

## 10 分钟长素材（v0.4.0）

TIME 新增 1/2/5/10 min，一次点击 START 连续录制到所选时长，不需要手动分段。
默认时长仍为 10 秒，已保存的曝光、增益和时长继续沿用；现场需要选择 TIME=10 min。
素材直接写入板端存储，不依赖热点持续传输；SSH 临时运行时不能据此假定断开连接后仍存活，
应使用桌面 APP 启动。正常结束后再通过 Wi-Fi 拉取。

录制核心与 UI 上限统一为 600 秒，发送日志容量按时长预分配，修复原 12000 帧限制。
开始前以 8 MiB/s 加预留空间估算需求，默认 10 分钟要求约 5.2 GiB 可用空间；
这是容量预算，不是保证码率。运行时每秒检查空间，低于 512 MiB 或查询失败则结束并保留失败记录。
帧索引和部分诊断元数据仍在正常退出时汇总，断电或强制杀进程不能保证完整元数据。

现场按 clip 编号记录场景切换的相对时间、距离、颜色、曝光和运动方式；
切分标注时保留原始视频及 CSV，并保持“子片段帧号 → 原片帧号/PTS”的映射。
同一长片及其所有切片应放在同一个训练/验证/测试分组，避免相邻帧泄漏。
10 分钟模式需完成板端长测后才能宣称已验证；构建和主机测试不能替代实机验证。

## 无屏 SSH 录制（v0.4.1）

在核心板 SSH 终端执行：

```bash
# 10 分钟、500 μs、1 倍模拟增益；立即开始后台录制
python3 /maixapp/apps/dart_data_recorder/recorderctl.py start \
  --seconds 600 --exposure-us 500 --gain 1 --label session01
# 查询服务和最近日志（包含本次目录）
python3 /maixapp/apps/dart_data_recorder/recorderctl.py status
# 提前正常结束，等待编码器排空、保存日志和恢复桌面
python3 /maixapp/apps/dart_data_recorder/recorderctl.py stop
```

start 将任务交给临时 systemd 服务，不依赖 SSH 会话且不配置开机自启。
无需屏幕或触摸确认，不读取/改写触屏保存参数；默认 600 秒、500 μs、1×，可通过命令明确设置。
`--gain` 在此命令中使用倍数（1～16），与底层 main.sh/direct_record 的 raw 值不同。
参数只在开始时生效，录制中不在线更改。录制前临时停止桌面服务，结束后按原状态恢复；
如有其他 MaixAPP 正在运行则拒绝开始，不强行关闭用户应用。

启动后应查询 status，确认进入录制；systemd 接受任务不等于相机初始化成功。
实际录制进度见日志给出的目录内 progress.json（约每 10 秒刷新）。
stop 向本服务的采集子进程发送 SIGINT，不使用 kill -9；提前结束时 capture.json 中
interrupted=true。保存成功不表示录满计划时长。不要断电等待保存。

### v0.4.2：长录像编码队列恢复

录制入口启用 `--venc-retry`。编码器队列短暂满时，对同一帧最多重试 200 ms；
仍未恢复或出现其他错误则停止并保留 `.failed` 证据。重试不保证源帧连续，
验收需同时检查 `capture.json` 和逐帧 CSV 的序号、时间戳与编码对应关系。
其他直接使用 `DirectVenc` 的调用保持默认 20 ms 重试预算。

### v0.4.3：每 30 秒主动落盘

录制入口默认每 30 秒刷新 `record.h264` 与 `encoded_frames.csv` 的用户态缓冲，
再在单独线程依次执行 `fdatasync`。同步成功后原子更新并同步
`durable_checkpoint.json` 和所在目录，其中记录已同步的完整帧数与文件字节下界。
正常退出时额外同步最后一批数据。同步错误会记录到 stderr 并导致录制失败。
使用原来的 SSH 录制命令即可，无需额外参数；视频保持单文件，不每 30 秒分段。

同步期间仍可能因存储竞争造成采集停顿；若上次同步未完成，不并发堆积同步任务。
因此“30 秒”是触发周期，不是断电最多丢 30 秒的硬保证。实际掉电行为取决于
文件系统和存储设备；需上板及断电测试确认。`frames.csv` 等退出时生成的汇总
不在周期同步范围内，异常退出时可参考已同步的编码索引与 checkpoint。
