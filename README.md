# ros2-latency-optimization

ROS 2 延迟优化研究:面向机器人图像、点云与控制链路,梳理延迟来源、可实施的优化方法、中间件选型与源码修改方向,并在 NVIDIA Jetson AGX Orin 上以真实传感器数据完成实测验证。

## 项目目的

ROS 2 系统的端到端延迟直接影响机器人感知与控制的实时性。本项目旨在回答三个问题:

1. **延迟来自哪里** —— 区分传输、排队、调度与计算,先测量、再逐项优化,而不是盲目调参;
2. **如何优化** —— 从应用结构、QoS、大消息(图像/点云)处理、内存与线程调度等层面给出可实施的方法,并分析各方法的适用边界;
3. **要不要换中间件或改源码** —— 比较常见中间件方案的适用场景,评估修改 ROS 2 源码的空间,明确"更换中间件或修改源码不保证降低端到端延迟"。

理论方法之外,项目在 Jetson AGX Orin 上使用 RealSense D455 相机与 RoboSense Airy 激光雷达的真实数据流,对共享内存中间件与 ROS 2 默认 Fast DDS 进行了同帧实测对比,作为方法论的实际验证。

## 内容

| 目录 | 内容 |
| --- | --- |
| [`docs/`](docs/) | 《ROS 2 延迟优化指南》全文、中间件调研、核心论文解读，以及《ROS 2 Jazzy 机内与机间通信数据链》源码级分析（`5 ROS2 Jazzy 通信数据链源码分析` 的 Markdown 与 Word 同名版本） |
| [`docs/3 七种通信中间件对比与选型.md`](<docs/3 七种通信中间件对比与选型.md>) | 七种中间件（Fast DDS / Cyclone DDS / Zenoh / iceoryx2 / eCAL / SIM / Agnocast）的定位矩阵、能力边界、工具可见性与选型建议（不含实测数据；Word 版同名） |
| [`docs/4 五种中间件延迟对比.md`](<docs/4 五种中间件延迟对比.md>) | 五种中间件（DAVOS_SIM / ROS2 / IOX2 / eCAL / Agnocast）同帧实测：测量条件、延迟分位数、漏帧率、CPU 与 RSS |
| [`docs/6 ROS 2 中间件机内延迟对比.md`](<docs/6 ROS 2 中间件机内延迟对比.md>) | 机内 ROS 2 中间件对比：Fast DDS（默认 / 仅 SHM / 仅 UDP）与 CycloneDDS 在相机、激光雷达、25 关节数据上的延迟，以及 Fast DDS Data-sharing 可用性验证 |
| [`docs/RESULTS_CROSS_MACHINE.md`](<docs/RESULTS_CROSS_MACHINE.md>) | Orin ↔ 5060Ti 工作站跨机实测：FastDDS 与 Zenoh 在相机、激光雷达、25 关节数据上的延迟与丢帧对比 |

> 原始 CSV、压测工具链与源码笔记不随本仓库提供；文档中的数字对应 2026 年 11 月在 AGX Orin 上的实测。

## 文档命名约定

`docs/` 下需要评审的正文文档统一为「序号 + 标题」命名，**同一份内容的 Markdown 源与 Word 版共用同一个文件名**（仅扩展名不同），并都放在 `docs/` 目录。序号 1、2.1、2.11 目前只有 Word，没有 Markdown 源；`4 五种中间件延迟对比.md` 与 `RESULTS_CROSS_MACHINE.md` 是纯 Markdown 的数据报告，没有 Word 版。

```text
docs/3 七种通信中间件对比与选型.md      /  .docx
docs/4 五种中间件延迟对比.md            （纯 Markdown，无 Word）
docs/5 ROS2 Jazzy 通信数据链源码分析.md  /  .docx
docs/6 ROS 2 中间件机内延迟对比.md       /  .docx
```

本仓库只保留 Markdown 与 Word 成稿，不包含格式转换脚本；文档插图统一放在 `docs/assets/`。

## License

[MIT](LICENSE)
