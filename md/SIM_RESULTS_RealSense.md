# D455 实测对比：2026-10-20

已在 NVIDIA Jetson AGX Orin Developer Kit 上，使用指定 `omni_sdk/sensing/realsense-ros` 驱动采集真实 D455 彩色图和深度图，再由 producer 直接读取同一份 ROS2 bag，比较 DAVOS 图像共享内存与 ROS2 默认 Fast DDS（`rmw_fastrtps_cpp`）。

录制约 35.41 秒：彩色图 1,062 帧、深度图 1,063 帧，均约 30 Hz。每种图像的四组实验分别选择该话题开头 30 秒的相同窗口，每组发送 900 帧。已通过独立脚本从原始 bag 重算输入帧数和时间序列签名，并核对全部 39,266 条接收记录的序号、尺寸、有效负载字节数和延迟计算；同组同帧在各 reader 中的发送时间一致，无重复或负延迟。

<!-- BEGIN UNIFIED RESULTS -->
## 延迟与漏帧

| 数据 | 路径 | 读者数 | 接收 / 应收 | 平均 ms | P50 ms | P95 ms | P99 ms | 最大 ms | 漏帧率 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| RealSense 彩色 rgb8 | DAVOS_SIM | 1 | 900 / 900 | 0.422 | 0.434 | 0.495 | 0.533 | 2.668 | 0.00% |
| RealSense 彩色 rgb8 | ROS2 | 1 | 868 / 900 | 1.468 | 1.508 | 1.624 | 1.804 | 3.574 | 3.56% |
| RealSense 彩色 rgb8 | DAVOS_SIM | 10 | 9,000 / 9,000 | 0.760 | 0.659 | 1.262 | 2.283 | 9.152 | 0.00% |
| RealSense 彩色 rgb8 | ROS2 | 10 | 8,821 / 9,000 | 3.437 | 3.293 | 4.437 | 4.875 | 6.848 | 1.99% |
| RealSense 深度 16UC1 | DAVOS_SIM | 1 | 900 / 900 | 0.286 | 0.286 | 0.340 | 0.378 | 1.883 | 0.00% |
| RealSense 深度 16UC1 | ROS2 | 1 | 882 / 900 | 1.152 | 1.139 | 1.393 | 1.435 | 1.893 | 2.00% |
| RealSense 深度 16UC1 | DAVOS_SIM | 10 | 9,000 / 9,000 | 0.543 | 0.462 | 0.987 | 2.015 | 6.047 | 0.00% |
| RealSense 深度 16UC1 | ROS2 | 10 | 8,895 / 9,000 | 2.428 | 2.353 | 3.233 | 3.671 | 6.345 | 1.17% |

平均值及 P50 / P95 / P99 / 最大值均由该组全部 reader 的逐帧样本合并计算。延迟只统计成功接收的帧，应结合漏帧率解读。

## 延迟降低百分比

降低百分比 = (ROS2 延迟 − DAVOS_SIM 延迟) / ROS2 延迟 × 100%。

| 数据 | 读者数 | 平均降低 | P50 降低 | P95 降低 | P99 降低 | 最大值降低 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| RealSense 彩色 rgb8 | 1 | 71.23% | 71.20% | 69.50% | 70.45% | 25.35% |
| RealSense 彩色 rgb8 | 10 | 77.89% | 79.98% | 71.57% | 53.16% | -33.64% |
| RealSense 深度 16UC1 | 1 | 75.16% | 74.88% | 75.60% | 73.70% | 0.55% |
| RealSense 深度 16UC1 | 10 | 77.62% | 80.37% | 69.46% | 45.11% | 4.68% |

按同一数据、相同读者数配对；百分比使用未舍入的原始统计值计算。正值表示延迟降低，负值表示延迟增加；ROS2 基线为 0 时显示 —。

## CPU 与内存

| 数据 | 路径 | 读者数 | producer CPU % | reader CPU 合计 % | CPU 合计 % | producer RSS MiB | 单 reader 平均 RSS MiB |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| RealSense 彩色 rgb8 | DAVOS_SIM | 1 | 11.77 | 3.68 | 15.45 | 32.80 | 9.15 |
| RealSense 彩色 rgb8 | ROS2 | 1 | 13.95 | 3.29 | 17.23 | 32.47 | 25.09 |
| RealSense 彩色 rgb8 | DAVOS_SIM | 10 | 15.63 | 71.27 | 86.90 | 32.82 | 9.12 |
| RealSense 彩色 rgb8 | ROS2 | 10 | 18.35 | 54.18 | 72.53 | 33.71 | 27.12 |
| RealSense 深度 16UC1 | DAVOS_SIM | 1 | 9.94 | 3.17 | 13.11 | 30.50 | 8.18 |
| RealSense 深度 16UC1 | ROS2 | 1 | 11.71 | 2.72 | 14.43 | 30.43 | 24.29 |
| RealSense 深度 16UC1 | DAVOS_SIM | 10 | 14.08 | 68.37 | 82.45 | 30.52 | 8.77 |
| RealSense 深度 16UC1 | ROS2 | 10 | 14.34 | 37.75 | 52.09 | 31.68 | 26.91 |

CPU 的 100% 表示一个逻辑核心；reader CPU 为各接收进程采样均值之和，CPU 合计为 producer 与 reader CPU 之和。producer 包含输入读取、解码和输出准备，reader 包含 CSV 写入；传感器驱动未纳入资源统计。

RSS 按 MiB（2^20 字节）换算，包含共享页；单 reader RSS 对各 reader 的时间均值再取平均，共享页会在多个进程中重复计入。

<!-- END UNIFIED RESULTS -->

## 结果解读

本次 DAVOS_SIM 的平均延迟及 P50 / P95 / P99 均较低，四组均无漏帧。10 读者的 reader CPU 较高；彩色 10 读者最大延迟由 ROS2 的 6.848 ms 增至 DAVOS_SIM 的 9.152 ms，对应最大值降低 −33.64%。这些数值来自本机每组 30 秒的一次测量。

![D455 平均延迟与 P99 对比](../../results/realsense/d455_comparison_20261020/latency_comparison.png)

[下载 PDF 图表](../../results/realsense/d455_comparison_20261020/latency_comparison.pdf)

## 测量条件

- 相机：RealSense D455，序列号 `263122301868`，固件 `5.15.1.55`，USB 3.2；RealSense ROS `4.59.1`，librealsense `2.59.1`。
- 彩色与深度均为 640×480、30 Hz，采集时同时启用。彩色为 `rgb8`，每帧 921,600 字节；深度为 `16UC1`，每帧 614,400 字节。未启用点云或深度对齐，原始深度值保持不变。
- 延迟使用主机单调时钟，从每条路径发送前到 reader 完成私有 OpenCV 图像拷贝结束；排除曝光、USB、驱动、输入 DDS、bag 解码、布局整理和预先构造 ROS2 输出消息的耗时。
- ROS2 输出使用 `best_effort`、`volatile`、`keep_last(1)`，采用默认传输，可能使用同机共享内存。DAVOS 保存最新帧，使用本次修复后的图像布局版本 2 和按槽进程共享读写锁，空闲轮询休眠 50 微秒。
- ROS2 Humble、Release 构建，`ROS_DOMAIN_ID=182`、`ROS_LOCALHOST_ONLY=1`。所有 reader 就绪后等待 2 秒供通信发现；首帧计入统计，资源采样间隔 0.2 秒。
- 每个 producer 在进程内读取相同 bag 的选定话题，按记录时间重放；彩色、深度分别从各自话题首帧起选取窗口。各组最大重放调度延后约 1.02–3.29 ms，逐组数值保存在 `producer.json` 和完整比较表。

## 数据与复现

- [相机与采集记录](../../results/realsense/d455_capture_20261020/capture.json)、[原始 bag 元数据](../../results/realsense/d455_capture_20261020/input/metadata.yaml)、[驱动日志](../../results/realsense/d455_capture_20261020/driver.log)
- [彩色完整报告](../../results/realsense/d455_color_30s_20261020/report.md)、[彩色逐 reader 统计](../../results/realsense/d455_color_30s_20261020/summary.csv)、[彩色运行参数](../../results/realsense/d455_color_30s_20261020/metadata.json)
- [深度完整报告](../../results/realsense/d455_depth_30s_20261020/report.md)、[深度逐 reader 统计](../../results/realsense/d455_depth_30s_20261020/summary.csv)、[深度运行参数](../../results/realsense/d455_depth_30s_20261020/metadata.json)
- [八组完整比较表](../../results/realsense/d455_comparison_20261020/comparison.csv)、[独立核验结果与 bag SHA256](../../results/realsense/d455_comparison_20261020/verification.json)、[核验与绘图脚本](../../results/realsense/d455_comparison_20261020/analyze.py)

本次正式实验的报告、逐帧 CSV、资源采样、采集元数据和图表已归档到 Git，可直接重新计算统计结果。原始 bag 保存在采集主机 `results/realsense/`，克隆项目后复现传输实验需先录制或拷贝对应 bag。复现实测使用：

```bash
cd /mnt/nvme_data/somin_xu/DAVOS_SIM
source /opt/ros/humble/setup.bash
source ros2_comp_ws/install/setup.bash
export ROS_DOMAIN_ID=182
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 examples/realsense/run_benchmark.py --stream color \
  --bag results/realsense/d455_capture_20261020/input --duration 30 --readers 1 10
python3 examples/realsense/run_benchmark.py --stream depth \
  --bag results/realsense/d455_capture_20261020/input --duration 30 --readers 1 10
```

已完成核心库和 ROS2 实验包构建、图像共享内存多进程回归、16 位合成数据同时收发验证、自动启动 D455 的实时彩色收发检查、上述八组同帧 bag 对比及独立数据核验。旧相机 reader 示例中的宽高构造参数也已修正并通过 C++11 编译。构建和重新采集步骤见[运行文档](README.md)。
