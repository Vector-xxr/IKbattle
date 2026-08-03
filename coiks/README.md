# CoIKS：并发逆运动学求解器

## 概述

CoIKS（Concurrent Inverse Kinematics Solver）是一套面向高效、稳健逆运动学（IK）求解的框架。它通过并行处理与多种 IK 策略，为机器人末端位姿生成多个解，并按预设准则选出最优解。该框架模块化、多线程，面向现代多核 CPU，可适配不同机器人应用。

## 主要特性

- **并行 IK 求解器**：同时运行多个 IK 求解器，提高找到可行且更优解的概率。
- **独立求解器**：模块化设计，支持多种 IK 求解方式：
  - **INVJ-RR**：基于雅可比的求解器，配合随机重启，处理关节限位与局部极小。
  - **QP 求解器**：基于二次规划，将关节限位作为不等式约束。
  - **NLO 求解器**：基于非线性优化，直接对目标位姿进行优化求解。
- **并发组合变体**：组合不同求解器（或同一求解器的多个实例），提升速度与可靠性：
  - **CoIKS-QP**：INVJ-RR + QP，增强鲁棒性。
  - **CoIKS-NLO**：INVJ-RR + NLO，兼顾约束处理。
  - **CoIKS-nINVJ**：多个 INVJ-RR 实例，使用不同随机种子。
- **求解模式**：三种选优方式：
  - **Speed（速度）**：优先取最先得到的可行解。
  - **Distance（距离）**：使关节相对初值变化最小。
  - **Manipulability（可操作度）**：使解处机器人运动裕度最大。
- **中文运行日志**：初始化、求解过程与统计信息等终端输出为中文；求解器名称等程序标识仍为英文（如 `coiks_invj`、`coiks_nlo`）。

## 快速开始

### 环境要求

- **操作系统**：Linux Ubuntu 20.04（也可用本仓库 `docker/` 在其他系统上运行）
- **编译器**：支持 C++14
- **ROS**：可与 ROS 集成，算法部分也可相对独立使用
- **依赖**：
  - [Pinocchio](https://github.com/stack-of-tasks/pinocchio)：运动学计算
  - [CasADi](https://web.casadi.org/)：非线性优化与 QP/HQP 后端
  - [IPOPT](https://coin-or.github.io/Ipopt/)：非线性优化（线性求解器可用 HSL/MA57；无授权时可用免费的 MUMPS）
  - [OSQP](https://osqp.org/)：QP 求解，支持 warm start

Docker 环境与编译说明见仓库根目录下的 [docker/README.md](../docker/README.md)。

三类底层 IK（INVJ / QP / NLO）的原理笔记见 [include/README.md](include/README.md)。

### 与基准测试的关系

本目录是 IK 求解库本身；批量评测、CSV 结果保存见 [coiks_benchmark/README.md](../coiks_benchmark/README.md)。  
Benchmark 会调用本库，并将每次实验结果写入 `coiks_benchmark/output/001`、`002`…。

## 许可证

本项目采用 GPL-3.0 许可证，详见 [LICENSE](../LICENSE) 文件。
