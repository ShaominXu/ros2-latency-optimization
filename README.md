# ros2-latency-optimization

ROS 2 中间件延迟优化研究:在 Jetson AGX Orin 上,对共享内存中间件 **DAVOS** 与 ROS 2 默认的 **Fast DDS**(`rmw_fastrtps_cpp`)进行真实传感器数据流的传输延迟实测对比。

## 项目简介

自动驾驶与机器人系统中,传感器数据(图像、点云)在多进程间的传输延迟直接影响感知链路的实时性。本项目基于论文 [A Faster and More Reliable Middleware for Autonomous Driving Systems](docx/论文解读A%20Faster%20and%20More%20Reliable%20Middleware%20for%20Autonomous%20Driving%20Systems.docx)(DAVOS)开展研究,包含:

- **研究资料**:ROS 2 六种通信中间件调研、核心论文解读(见 `docx/`)
- **实测对比**:在 NVIDIA Jetson AGX Orin Developer Kit 上,使用 RealSense D455 相机与 RoboSense Airy 激光雷达的真实数据,对两种中间件进行同帧对比实验(见 `md/`)

## 核心结论

在 1 / 10 读者、相机与激光雷达全部六组实验中,DAVOS 相比 ROS 2 默认 Fast DDS:

- **平均延迟降低约 71%–78%**,P99 延迟降低约 45%–74%
- **漏帧率从 1.17%–13.94% 降至 0.00%**(全部六组零漏帧)
- **单 reader 内存占用约 8–10 MiB**,为 ROS 2(约 24–28 MiB)的三分之一左右

## 实测结果汇总

延迟为 30 秒同帧对比窗口的统计,只计成功接收的帧,应结合漏帧率解读。完整数据见 [RealSense 报告](md/SIM_RESULTS_RealSense.md) 与 [RoboSense 报告](md/SIM_RESULTS_RoboSense.md)。

| 传感器数据 | 读者数 | ROS2 平均 ms | DAVOS 平均 ms | 平均降低 | P99 降低 | ROS2 漏帧率 | DAVOS 漏帧率 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| RealSense D455 彩色 rgb8 | 1 | 1.468 | 0.422 | 71.23% | 70.45% | 3.56% | 0.00% |
| RealSense D455 彩色 rgb8 | 10 | 3.437 | 0.760 | 77.89% | 53.16% | 1.99% | 0.00% |
| RealSense D455 深度 16UC1 | 1 | 1.152 | 0.286 | 75.16% | 73.70% | 2.00% | 0.00% |
| RealSense D455 深度 16UC1 | 10 | 2.428 | 0.543 | 77.62% | 45.11% | 1.17% | 0.00% |
| RoboSense Airy 点云 XYZ | 1 | 1.655 | 0.482 | 70.91% | 49.25% | 2.65% | 0.00% |
| RoboSense Airy 点云 XYZ | 10 | 3.791 | 0.901 | 76.22% | 49.95% | 13.94% | 0.00% |

注意:10 读者组的最大延迟出现个别反转(如彩色 10 读者时 DAVOS 为 9.152 ms,ROS2 为 6.848 ms),逐项数值见各报告。

## 实验环境

| 项目 | 配置 |
| --- | --- |
| 硬件 | NVIDIA Jetson AGX Orin Developer Kit |
| 系统 | ROS 2 Humble(Release 构建),`ROS_LOCALHOST_ONLY=1` |
| 基线 | Fast DDS(`rmw_fastrtps_cpp`),输出 QoS:`best_effort` / `volatile` / `keep_last(1)` |
| 对比对象 | DAVOS(修复后版本 2 布局,按槽进程共享读写锁,空闲轮询休眠 50 µs) |
| 相机 | Intel RealSense D455(640×480 @ 30 Hz,彩色 rgb8 / 深度 16UC1),librealsense 2.59.1 |
| 激光雷达 | RoboSense Airy(XYZ 点云,每点 16 字节),按 bag 记录时间重放 |

## 测量口径

- 延迟从 producer 发送前开始,到 reader 完成私有数据拷贝(图像 OpenCV / 点云 PCL)结束;不含传感器采集、驱动、bag 解码耗时
- producer 在进程内直接读取同一份 ROS 2 bag 重放,保证两种中间件收到完全相同的输入帧
- 输入帧数、时间序列签名、全部接收记录均经独立脚本核验
- 每组为本机 30 秒窗口的一次测量

## 目录结构

```
├── docx/                                  研究资料与论文解读
│   ├── ROS2_Latency_Optimization.docx         项目总览
│   ├── ROS 2 六种通信中间件研究资料汇总.docx     中间件调研
│   ├── 论文解读Latency Analysis of ROS2 Multi-Node Systems.docx
│   ├── 论文解读A Faster and More Reliable Middleware for Autonomous Driving Systems.docx
│   ├── 中间件SIM实验——RealSense D455相机.docx   实验设计与结果(D455)
│   ├── 中间件SIM实验——RoboSense Airy激光雷达.docx 实验设计与结果(Airy)
│   └── 最值得看的论文.docx                      延伸阅读清单
├── md/                                    实测结果报告(Markdown)
│   ├── SIM_RESULTS_RealSense.md               D455 相机实测报告
│   └── SIM_RESULTS_RoboSense.md               Airy 激光雷达实测报告
├── LICENSE
└── README.md
```

## 复现

实验主机上的构建、实时传感器接入与重新采集步骤,以及逐帧核验方法,见各报告的「测量条件」与「数据与复现」章节([RealSense](md/SIM_RESULTS_RealSense.md) / [RoboSense](md/SIM_RESULTS_RoboSense.md))。核心流程:

```bash
source /opt/ros/humble/setup.bash
source ros2_comp_ws/install/setup.bash
export ROS_DOMAIN_ID=181   # D455 实验为 182
export ROS_LOCALHOST_ONLY=1
python3 examples/rslidar/run_benchmark.py \
  --bag results/rslidar/airy_capture_20261019_161049/input \
  --duration 30 --readers 1 10
```

## 相关文献

- *A Faster and More Reliable Middleware for Autonomous Driving Systems*(DAVOS)
- *Latency Analysis of ROS 2 Multi-Node Systems*

完整解读与延伸阅读见 [`docx/`](docx/) 目录。

## License

[MIT](LICENSE)
