# CoIKS（并发逆运动学求解器）

## 概述

CoIKS（Concurrent Inverse Kinematics Solver）是一套通过多求解器并发计算来高效求解逆运动学（IK）问题的框架。本仓库包含 CoIKS 核心实现、基准测试工具，以及用于对比的 TRAC-IK 集成，便于在多种机器人模型上进行性能比较与使用。

## 目录结构

本仓库主要包含以下目录：

- **coiks/**：CoIKS 求解器核心实现，通过并发计算求解逆运动学。目录内另有独立 README，介绍功能、依赖、使用方法以及求解器架构与配置等技术细节。
- **coiks_benchmark/**：路径跟踪基准与可视化，用于将 CoIKS 与 TRAC-IK 等对比。覆盖多种机器人模型（含 OpenArm）；运行日志为中文，结果 CSV 写入包内 `output/001`、`002`… 目录。详见 [coiks_benchmark/README.md](coiks_benchmark/README.md)。
- **trac_ik_lib/**：TRAC-IK 子模块，用于在基准测试中与 CoIKS 对比。使用前需初始化该子模块以获取 TRAC-IK 功能。（若使用本仓库 Docker 环境，也可直接依赖镜像内的 `ros-noetic-trac-ik`，不必初始化此子模块。）
- **docker/**：Ubuntu 20.04 + ROS Noetic 的 Docker 配置与使用说明，适合在 Ubuntu 22.04 等宿主机上运行本项目。详见 [docker/README.md](docker/README.md)。



## 运行说明（本仓库定制）

- **终端输出**：核心库与 benchmark 的运行日志已改为中文（求解器名等配置标识仍保持英文，如 `coiks_invj_multi`）。
- **实验结果**：每次 `roslaunch coiks_benchmark ...` 会在  
`coiks_benchmark/output/` 下新建编号目录（`001`、`002`、…），同一次实验的各求解器 CSV 放在同一目录。  
Docker 环境下该路径对应宿主机  
`/path/to/CoIKS/coiks_benchmark/output/`。

