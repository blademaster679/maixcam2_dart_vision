# 项目锚点与已知边界

核验基线为2026-09-08。路径用于定位，执行前核对文件、版本和当前设备占用，不重新做已经保存的整轮搜索。

| 项目 | 锚点 |
| --- | --- |
| 用户项目 | `/home/blade_master/pnx/maixcam2_dart_vision` |
| MaixCDK | `/home/blade_master/maix/MaixCDK`，此前commit `2a0502ecb20e5695b28580b3689492b7a228f9e4` |
| MSP | `3.0.0_20250319114413`，Axera/AX630C平台 |
| 设备连接 | 项目 `.maixpy/` 配置，MaixPy helper；此前有线地址10.18.197.1 |
| MaixPy使用方法 | `/home/blade_master/pnx/maixpy-skill/maixpy/SKILL.md`，连接前读其workflow引用；不在skill里存凭据 |
| 芯片 | 读回ID `0x530441`，I²C 0x36，XCLK24MHz |
| 原系统库 | `/opt/lib/libsns_os04a10.so`，SHA256 `25f1d2c4e9fdef213dca3fdecf09f98a61d9a52c0704ade31cf6070aeab5bb4c` |
| Axera传感器源码 | SDK的 `dl/extracted/maixcam2_msp_srcs/maixcam2_msp_arm64_glibc_v3.0.0_20250319114413/component/isp_proton/sensor/ov_os04a10/` |
| settings源哈希 | `e469a1321125685357202e211cd52c685e92f9929a0de576079c92d01314949f` |
| 当前工具 | 项目 `tools/os04a10_highfps/` |
| 公开资料与源码清单 | 工具目录 `FULL_FOV.md`、`fullfov_sources.json`；详细数据 `.maixpy/fullfov/` |

已完成对照：

- 全幅30fps：原生2688×1520；阵列X=0..2703、Y=0..1535、输出偏移9/9，采样候选地址0x3814..0x3817均读回1。初期假设实验优先使用该模式和原厂模拟初始化。
- 全幅90fps：108MHz SCLK、HTS732、VTS1640、PLL1=0x5c、PLL2=0xd8，30秒2699帧、约89.958fps；构建 `.maixpy/fullfov/native90-readback/`，不是降采样。
- 中心裁剪：640×360、108MHz、HTS732、VTS410，RAW30分钟平均359.832fps，NV21短测通过；构建 `.maixpy/crop360-final/`。只有约24%原图宽/高，不能冒充完整视野。
- 已检查8份同型号源码/库的76段序列，包括部分表和跨仓库重复。没有找到全视野binning/skipping完整序列。`source_inventory.py`可复现离线分析。

关键资料在项目 `.maixpy/datasheets/` 的OS04A10 rev1B PDF/text。页31～35为窗口、翻转和时序；页40为PLL；页59为模式摘要。页59的360fps包含binning和裁剪，不能推出全视野360fps可行。字面地址相近不能证明兼容其他传感器的位定义。

当前实验边界采用资料已有额定上限SCLK108MHz、PCLK138MHz、MIPI1104Mbps/lane；初期不改变时钟。此前以内部温度80°C作为停止值，来自85°C额定结温上限预留余量。该测温未外部校准，不能排除未知寄存器的额外副作用。

全部实验源码和二进制置于项目 `.maixpy/` 独立目录；可维护代码位于工具目录。不修改共享SDK作为唯一实现，不接入 `main/src/` 检测业务。
