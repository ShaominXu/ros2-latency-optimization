# Airy 实测对比：2026-10-19

在 NVIDIA Jetson AGX Orin Developer Kit 上，使用指定 Omni SDK 中的 `RSAIRY` 驱动采集真实点云，再由 producer 直接读取同一份 bag，分别比较 DAVOS 与 ROS2 默认 Fast DDS（`rmw_fastrtps_cpp`）。

共录得约 33 秒、333 帧。每组选择开头 30 秒的同一窗口，共 302 帧；四组的输入帧数、首末记录时间、时间序列签名及发送帧数均一致。输入中 240 帧为 86,400 点，其余帧为 82,560、82,656 或 86,304 点，按原始点数保留。另行读取原始 bag，核对了全部 6,215 条成功接收记录的点数与输入帧对应一致，且没有负延迟。

<!-- BEGIN UNIFIED RESULTS -->
## 延迟与漏帧

| 数据 | 路径 | 读者数 | 接收 / 应收 | 平均 ms | P50 ms | P95 ms | P99 ms | 最大 ms | 漏帧率 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| RSLiDAR XYZ | DAVOS_SIM | 1 | 302 / 302 | 0.482 | 0.451 | 0.571 | 1.471 | 3.215 | 0.00% |
| RSLiDAR XYZ | ROS2 | 1 | 294 / 302 | 1.655 | 1.560 | 2.223 | 2.898 | 3.713 | 2.65% |
| RSLiDAR XYZ | DAVOS_SIM | 10 | 3,020 / 3,020 | 0.901 | 0.744 | 1.967 | 3.204 | 11.579 | 0.00% |
| RSLiDAR XYZ | ROS2 | 10 | 2,599 / 3,020 | 3.791 | 3.737 | 5.563 | 6.402 | 9.267 | 13.94% |

平均值及 P50 / P95 / P99 / 最大值均由该组全部 reader 的逐帧样本合并计算。延迟只统计成功接收的帧，应结合漏帧率解读。

## 延迟降低百分比

降低百分比 = (ROS2 延迟 − DAVOS_SIM 延迟) / ROS2 延迟 × 100%。

| 数据 | 读者数 | 平均降低 | P50 降低 | P95 降低 | P99 降低 | 最大值降低 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| RSLiDAR XYZ | 1 | 70.91% | 71.08% | 74.29% | 49.25% | 13.41% |
| RSLiDAR XYZ | 10 | 76.22% | 80.10% | 64.65% | 49.95% | -24.95% |

按同一数据、相同读者数配对；百分比使用未舍入的原始统计值计算。正值表示延迟降低，负值表示延迟增加；ROS2 基线为 0 时显示 —。

## CPU 与内存

| 数据 | 路径 | 读者数 | producer CPU % | reader CPU 合计 % | CPU 合计 % | producer RSS MiB | 单 reader 平均 RSS MiB |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| RSLiDAR XYZ | DAVOS_SIM | 1 | 8.23 | 2.88 | 11.11 | 37.58 | 9.66 |
| RSLiDAR XYZ | ROS2 | 1 | 8.56 | 1.21 | 9.77 | 34.45 | 26.89 |
| RSLiDAR XYZ | DAVOS_SIM | 10 | 8.43 | 41.75 | 50.18 | 34.61 | 9.84 |
| RSLiDAR XYZ | ROS2 | 10 | 9.80 | 15.29 | 25.09 | 35.68 | 28.27 |

CPU 的 100% 表示一个逻辑核心；reader CPU 为各接收进程采样均值之和，CPU 合计为 producer 与 reader CPU 之和。producer 包含输入读取、解码和输出准备，reader 包含 CSV 写入；传感器驱动未纳入资源统计。

RSS 按 MiB（2^20 字节）换算，包含共享页；单 reader RSS 对各 reader 的时间均值再取平均，共享页会在多个进程中重复计入。

<!-- END UNIFIED RESULTS -->

## 结果解读

本次平均延迟在 1 / 10 读者时分别降低 70.91% / 76.22%，P99 分别降低 49.25% / 49.95%。10 读者最大延迟由 ROS2 的 9.267 ms 增至 DAVOS_SIM 的 11.579 ms，对应最大值降低 −24.95%；10 读者的 reader CPU 占用也更高。这些数值来自本机每组 30 秒的一次测量。

## 测量条件

- 有效负载为 XYZ，每点 16 字节，保留 NaN；原始 intensity、ring 和逐点时间不参与对比。
- 延迟从发送前主机系统时间开始，到 reader 完成私有 PCL 点云拷贝结束，不包含雷达采集、原始驱动、bag 读取和字段转换。
- 延迟统计只包含成功接收的帧，应与漏帧率一起解读。
- ROS2 输出 QoS 为 `best_effort`、`volatile`、`keep_last(1)`；DAVOS 保留最新帧。ROS2 使用默认传输，未强制为纯 UDP。
- DAVOS 使用修复后的版本 2 点云布局及按槽进程共享读写锁，容量为每帧 1,000,000 点，空闲轮询休眠为 50 微秒。
- ROS2 Humble，Release 构建，`ROS_DOMAIN_ID=181`、`ROS_LOCALHOST_ONLY=1`，预热 2 秒，资源采样间隔 0.2 秒。
- 这是本机每组 30 秒的一次测量。按 bag 记录时间重放，最大调度延后约 8.75–12.72 毫秒，逐组数值记录在 `producer.json`。

## 数据与复现

- [驱动采集记录](../../results/rslidar/airy_capture_20261019_161049/capture.json)
- [原始 bag 元数据](../../results/rslidar/airy_capture_20261019_161049/input/metadata.yaml)
- [完整汇总报告](../../results/rslidar/20261019_161937_585392/report.md)
- [逐 reader 统计与内存数据](../../results/rslidar/20261019_161937_585392/summary.csv)
- [运行参数和完成状态](../../results/rslidar/20261019_161937_585392/metadata.json)

本次正式实验的报告、逐帧 CSV、资源采样和采集元数据已归档到 Git，可直接重新计算统计结果。原始 bag 保存在采集主机 `results/rslidar/`，克隆项目后复现传输实验需先录制或拷贝对应 bag。早先通过 `ros2 bag play` 的共同 DDS 首跳进行的排查结果因各组输入帧数不同，未纳入本表；本表使用进程内直接读取 bag 的复测结果。

```bash
cd /mnt/nvme_data/somin_xu/DAVOS_SIM
source /opt/ros/humble/setup.bash
source ros2_comp_ws/install/setup.bash
export ROS_DOMAIN_ID=181
export ROS_LOCALHOST_ONLY=1
python3 examples/rslidar/run_benchmark.py \
  --bag results/rslidar/airy_capture_20261019_161049/input \
  --duration 30 --readers 1 10
```

构建、实时雷达接入及重新采集方法见[运行文档](README.md)。本次已完成构建、点云核心多进程回归测试、合成数据的独立 / 同时收发验证，以及上述实测 bag 的四组对比。
