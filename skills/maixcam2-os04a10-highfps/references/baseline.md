# 证据与当前基线

核验日期：2026-09-07。本文件记录设计 skill 时的观察，不代表之后板端的实际状态。

## 2026-09-08全视野研究更新（优先于下方记录）

- 用户要求结合资料和可获取代码尝试不裁剪的360p@360fps。检索并下载Rockchip 4.19/5.10/6.1、Luckfox、OpenIPC同型号源码、Axera AX620E静态库，结合本机Axera/Sophgo源码，共分析8份、76段序列（含部分表和跨仓库重复）。已找到的0x3814..0x3817显式写入均为1，没有得到可追溯的全视野binning/skipping序列，未猜写未知采样位。
- 项目新增 `source_inventory.py`（离线C/ELF数组分析）、`fullfov_sources.json`（来源及哈希）、`fullfov.py`（受限的原生全幅30/60/90时序对照），完整报告 `tools/os04a10_highfps/FULL_FOV.md`。
- 全幅30/60/90档5秒实测30.28144/59.97121/89.95476fps；90档30秒2699帧、89.95782885fps，序号/PTS异常、取帧/归还/MIPI错误均0，SSH与进程正常返回0。记录 `.maixpy/runs/os04a10-20260908-112247-efd79f/`，构建 `.maixpy/fullfov/native90-readback/`。
- 该对照保留原厂X=0..2703、Y=0..1535、输出2688×1520、偏移9/9、RGGB和采样字段。90档SCLK108MHz、HTS732、VTS1640、PLL2=0xd8，PLL1=0x5c来自同型号四lane HDR表，HDR自身禁用。4×720Mbps无法容纳全幅90fps RAW10，改用已有1104Mbps/lane配置；读回与预期一致，未做独立电气测量。
- **不裁剪640×360@360fps仍未实现。** 全幅90fps仅是RAW短测对照，未完成长测、NV21、公共Camera API或binning。保留全视野的有效4倍降采样约672×380只是数学研究目标，不能当作芯片已支持的模式。
- 模式选择及采集尺寸改为生成宏，保留原裁剪配方。裁剪回归5秒1799帧、约359.786fps，正常退出。全幅需要 `--fullfov-probe`、队列4，使用原厂池尺寸。部署仍为/tmp隔离，系统相机库不覆盖。

## 同日逆向开发更新（优先于下方早期记录）

- 已通过用户提供的MaixPy skill连接有线 `10.18.197.1`，MaixCAM2/AX630C，系统 `maixcam2-2026-05-29-maixpy-v4.12.5`，MSP `3.0.0_20250319114413`。回调读回chip ID `0x530441`，I²C 0x36，MCLK 24MHz。
- 用户明确授权通过已有代码逆向尝试。分析25张OS04A10模式表后，使用同型号既有PLL2字段及规格书窗口/时序定义，得到**纯中心裁剪640×360**：SCLK108MHz、HTS732、VTS410，名义359.856fps。没有任意扫描模拟寄存器，也没有获得厂商2×2 binning序列。
- 板端RAW完成30分钟采集：647,698帧、359.8321205fps；179个完整10秒窗口均359.8～359.9fps，序号丢失/重复/倒退、PTS不递增、取帧/归还错误和每秒MIPI ErrorCount均0；内部温度最高68.2549°C。
- 初次队列4的长测32秒发生一次跳号并停止。改为队列16、RAW池24帧后取得上述结果。RSS22,632～53,028KB包含保留的有界帧元数据。
- 长测采集JSON状态0，但主机SSH1835秒等待超时，最终进程退出码未知；确认采集进程消失、启动器恢复后补取全部CSV。新runner增加keepalive、远端输出重定向与独立进程退出码；10秒RAW复测3598帧且SSH/进程均返回0。
- 视野覆盖原图宽、高各约24%，面积约5.6%；用户尚未确认该视野是否满足需求。不能称为厂商binning模式或保留原视野。最新Bayer声明由镜像和偏移奇偶推导为BGGR，未完成色卡标定；30分钟RAW测试声明为RGGB，sensor库二进制相同。
- 源码、配方、构建与验收说明位于项目 `tools/os04a10_highfps/REVERSE_ENGINEERING.md`，该报告为最新结果入口。完整数据在 `.maixpy/runs/os04a10-20260907-214557-2035d1/`，已验证库在 `.maixpy/crop-359-buffered/`，最终构建 `.maixpy/crop360-final/`。
- 系统库没有覆盖，原SHA256为 `25f1d2c4e9fdef213dca3fdecf09f98a61d9a52c0704ade31cf6070aeab5bb4c`。实验库SHA256为 `07119dee1a09eb8aee86a5ed7b649ddf4f3edc296a0595011dc5b2c382a52395`。共享SDK和 `main/src/` 未改。
- NV21单独30秒通过：10,795帧、359.8250678fps，无序号/PTS/取帧/归还/MIPI错误，SSH和进程返回0，样帧640×360格式4。NV21队列16在SetChnAttr返回非法参数，改为4成功；RAW默认仍16。记录 `.maixpy/runs/os04a10-20260907-222820-54f0d8/`。
- 公共Camera API、受控动态场景、色卡与完整重启恢复尚未验收，NV21尚未做30分钟测试。缺厂商profile时默认构建仍拒绝HFR；使用可追溯 `--recipe` 构建中心裁剪候选。

## 用户与项目背景

- 用户表示已经有线连接 MaixCAM2，并换装 OS04A10。
- “查询Maixcam2相机型号”对话正文未出现在当前上下文；没有声称已读取该对话。
- 项目 `/home/blade_master/pnx/maixcam2_dart_vision/README.md` 的“OS04A10 360 FPS 独立研发线”记录了 RAW/NV21、模式资料和分阶段验收的初步方案。
- 旧项目记录是 OS04D10、系统 `maixcam2-2026-05-29-maixpy-v4.12.5`、MaixPy `4.12.5`；这些不是现在 OS04A10 的实测基线。
- 暂定 360p 为 640×360，优先帧率且允许 sensor ROI 裁剪；视野偏好未确认，模式论证中同时评估保留视野方案。
- 对历史 USB 地址 `root@10.18.197.1` 的只读 SSH 尝试得到 `Permission denied (publickey,password)`。这只能证明该地址 SSH 服务响应，不能证明已登录或核验了设备身份。需使用用户现有连接方式，不猜密码、不收集私钥。

## 官方资料

1. [Sipeed MaixCAM2 相机说明](https://wiki.sipeed.com/hardware/zh/maixcam/maixcam2_camera_lens.html)：列出 OS04A10 支持和 2688×1520@30fps 常用规格，不能据此认定低分辨率上限为 30fps，也没有列出目标 360fps 模式。
2. [OMNIVISION OS04A10 产品简表 v1.2](https://www.ovt.com/wp-content/uploads/2023/06/OS04A10-PB-v1.2-WEB.pdf)：支持裁剪、最多四 lane CSI-2，属于 rolling shutter；未给出 640×360@360fps 寄存器方案。1520p 的多曝光传输标注不能直接作为独立线性图像帧率。
3. [MaixCAM2 硬件规格](https://wiki.sipeed.com/maixcam2)：板卡采用 AX630C，摄像头接口为四 lane MIPI CSI、22 pin。SDK 中出现 `AX620E` 名称并不意味着本板型号已经由该名字得到验证。

完整 datasheet、mode application note、模组原理图与厂商实际寄存器表优先于公开简表。若后续资料推翻当前判断，更新证据与结论。

## 已核验的本地源码

SDK 根目录：`/home/blade_master/maix/MaixCDK`。
Git commit：`2a0502ecb20e5695b28580b3689492b7a228f9e4`，检查时工作区干净。
项目 `build/config/global_config.h` 选择 MSP `3.0.0_20250319114413`。

以下均相对于 SDK 根目录：

| 入口 | 已观察内容 / 开发用途 |
| --- | --- |
| `components/vision/port/maixcam2/maix_camera_maixcam2.cpp` | `Camera::open` 调用 `get_vi_case`，AI-ISP 读取系统配置；约 760 行的 `set_fps` 只调用 exposure，随后 `_fps` 被设成曝光时间倒数，不能视作帧率实测 |
| `components/maixcam_lib/include/maixcam2/ax_middleware.hpp` | `__get_vi_case` 中 OS04A10 固定为 2688×1520、30fps；定位 sample case、pool、VIN 和模式分发 |
| `components/3rd_party/maixcam2_msp/msp/sample/common/common_isp.c` | 传感器库名 `libsns_os04a10.so`，线性对象符号 `gSnsos04a10Obj`；核对动态加载和注册 ABI |
| `components/3rd_party/maixcam2_msp/CMakeLists.txt` | 实际 MSP 源码/库根由 `DL_EXTRACTED_PATH` 与版本拼接，不是仅在 component 的 `msp` 子目录中 |
| `dl/extracted/maixcam2_msp_srcs/maixcam2_msp_arm64_glibc_v3.0.0_20250319114413/component/isp_proton/sensor/ov_os04a10/` | 已找到 Axera 原生 `os04a10.c`、`os04a10_settings.h`、`os04a10_ae_ctrl.c`、Makefile 等，不应误判为只能拿到预编译驱动 |
| 同上 `os04a10_settings.h` | 枚举和数组含全幅 30fps、带 qs 注释的 60fps、HDR 等；尚未确认可用 360p@360fps 表。存在名为 `...HDR_250fps`、注释却写 25fps 的数组，不能按名字当高速模式证据 |
| `components/3rd_party/sophgo-middleware/.../sensor/sg200x/ov_os04a10/` | 另一平台的同名驱动，不能直接用于 Axera；即使参考寄存器，也需核验模式与板级条件 |

README 提到的 MaixCDK `dev` OS04D10 360p@120fps 只是后续对比线索；本次未核验远端最新实现，不把它当本地已有能力。

## 下一轮只读采集内容

使用确认的主机/用户登录，记录 `uname -a`、系统镜像标识、MaixPy 包版本、实际 sensor 库路径及哈希、相关进程和占用、设备树板卡标识、现有 sensor 日志。SDK 和板端的 MSP/ABI 需匹配。

读取已存在日志和状态，不启动会自动初始化 sensor 的 Camera 对象来冒充只读检查。Chip ID 查询要先确认手册地址和读取方式，不用无差别 `i2cdetect`/寄存器遍历。仅 I²C 从地址或库文件名不是身份确证。
