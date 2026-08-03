# CoIKS-Benchmark

用于在 15 种商业机械臂上对比 CoIKS、TracIK 与 Orocos KDL 的综合基准测试套件。

## 概述

本基准测试在统一条件下评估不同 IK 求解器的性能，包括：

- 每台机器人 10,000 个随机目标位姿
- 10,000 ~~~~个随机初始关节配置
- 完整 6D 位姿约束（位置 + 姿态）
- 通过正运动学生成可达目标位姿
- 标准化测试条件，保证对比公平
- **终端输出为中文**（求解器名等标识仍为英文）
- **结果 CSV** 写入本包 `output/` 下按实验编号递增的子目录

## 目录与代码说明

```
coiks_benchmark/
├── CMakeLists.txt              # catkin 编译配置，生成两个可执行文件
├── package.xml                 # ROS 包元数据与依赖声明
├── README.md
├── src/
│   ├── benchmark.cpp           # 随机位姿 IK 对比主程序
│   ├── benchmark_path.cpp      # 路径跟踪 IK 对比主程序
│   └── experiment_output.hpp   # 自动创建 output/001、002… 并拼 CSV 路径
├── scripts/
│   ├── viz_urdf.py             # Pinocchio + MeshCat 可视化 URDF
│   └── requirements-viz.txt    # 可视化脚本 Python 依赖
├── launch/
│   ├── coiks_benchmark.launch       # 启动随机位姿测试
│   └── coiks_benchmark_path.launch  # 启动路径跟踪测试
├── data/                       # 预生成的测试数据（CSV）
├── output/                     # 实验结果；每次运行新建 001/、002/…
└── urdf/                       # 各机器人 URDF 模型
```



### 源码（`src/`）


| 文件                      | 作用                                                                                                 |
| ----------------------- | -------------------------------------------------------------------------------------------------- |
| `benchmark.cpp`         | 主基准程序。封装 KDL、TRAC-IK、CoIKS；按 `robot_id` 加载 URDF 与输入 CSV；统计成功率与耗时，并写出结果 CSV。编译产物：`coiks_benchmark`。 |
| `benchmark_path.cpp`    | 路径跟踪基准。读取 `path_robot_*.csv`，对比 CoIKS 与 TRAC-IK。编译产物：`coiks_benchmark_path`。                       |
| `experiment_output.hpp` | 内部工具：在 `output/` 下创建下一个实验目录（`001`、`002`…），生成结果文件路径。                                                |




### Launch（`launch/`）


| 文件                            | 作用                                                                                     |
| ----------------------------- | -------------------------------------------------------------------------------------- |
| `coiks_benchmark.launch`      | 配置并启动随机位姿测试。可设 `robot_id`（0–14）、`ik_solver`、时限/误差、是否启用 TRAC-IK/KDL，以及 INVJ/NLO/QP 等参数。 |
| `coiks_benchmark_path.launch` | 配置并启动路径跟踪测试，参数含义类似，额外使用路径数据。                                                           |




### URDF 可视化（`scripts/`）

用 **Pinocchio + MeshCat** 回放基准 CSV（与 C++ 解耦，不必改 launch）。脚本会把 URDF 里旧的绝对 mesh 路径改写到本仓库 `urdf/`。

| `--mode` | 数据 | 行为 |
|----------|------|------|
| `csv` | `data/path_robot_<robot_id>.csv` | 沿路径点连续运动（简易 IK，种子为 `qinit[0]`） |
| `random` | `qinit_*` + `targets_*` | 从 `--index` 起播 `--count` 道题：先 `q_init` 再 `target`（瞬切）；题内间隔 `--dt`，跨题间隔 `2*dt` |
| `neutral` | 无 | 中性位静态 |

参数：`--index` 起始下标，`--count` 连续条数（`csv` 默认最多 200，`random` 默认最多 20），`--dt` 时间间隔（`csv` 默认 0.03，`random` 默认 0.5）。

```bash
cd coiks_benchmark
pip install -r scripts/requirements-viz.txt   # 首次：meshcat + pin

# 路径跟踪数据（对应 coiks_benchmark_path.launch）
python3 scripts/viz_urdf.py --robot_id 7 --mode csv --index 0 --count 200

# 随机 IK 数据（对应 coiks_benchmark.launch）：init/target 交叉并标注
python3 scripts/viz_urdf.py --robot_id 7 --mode random --index 10 --count 5 --dt 0.5

python3 scripts/viz_urdf.py --robot_id 1 --mode neutral
```

说明：Docker 镜像侧重 C++ Pinocchio，不一定自带 Python 绑定；可视化建议在宿主机运行。

常用启动：

```bash
roslaunch coiks_benchmark coiks_benchmark.launch
roslaunch coiks_benchmark coiks_benchmark_path.launch
```

通过改 launch 中的 `ik_solver` 切换 **CoIKS** 变体，例如：`coiks_invj_multi`、`coiks_nlo`、`coiks_qp`、`coiks_invj_nlo` 等。  
`kdl_enable` / `tracik_enable` 控制是否同时跑对比基线 KDL、TRAC-IK（它们不是 CoIKS 内部目录，但会各自写一份 CSV）。

### 数据（`data/`）


| 文件模式                                           | 作用                                    |
| ---------------------------------------------- | ------------------------------------- |
| `qinit_robot_<name>_random_10000samples.csv`   | 随机初始关节配置（每台机器人 1 万组）                  |
| `targets_robot_<name>_random_10000samples.csv` | 对应可达目标位姿（由 FK 生成，保证可达）                |
| `path_robot_<id>.csv`                          | 路径跟踪用的连续目标序列（`id` 对应 `robot_id` 0–14） |




### 结果（`output/`）

每次完整 `roslaunch` 运行会在 `output/` 下新建编号目录，同一次实验的所有求解器结果放在同一目录：

```text
coiks_benchmark/output/
├── 001/
│   ├── kdl_r1.csv                 # 若开启 kdl_enable
│   ├── trac_ik_r1.csv             # 若开启 tracik_enable
│   └── coiks_invj_multi_r1.csv    # 名称随 ik_solver 变化
├── 002/
│   └── ...
└── .gitkeep
```

- **目录编号**：`001`、`002`、`003`…，按已有目录自动递增。
- **文件名**：`{求解器名}[_path]_r{robot_id}.csv`
  - 求解器名：`kdl` / `trac_ik` / `coiks_invj_multi` 等（来自 `get_solver_name()`）
  - `_path`：仅路径跟踪基准有此后缀
  - `r{robot_id}`：对应 launch 中的机器人编号（如 `r1` = VS050）
- Docker 下容器路径 `/root/catkin_ws/src/coiks_benchmark/output/` 即宿主机仓库中的本目录。



#### CoIKS 结果 CSV 列含义


| 列名          | 含义                      |
| ----------- | ----------------------- |
| `idx`       | 样本编号                    |
| `res`       | `0` 成功，非 `0` 失败         |
| `time`      | 耗时（微秒）                  |
| `solver_id` | 并发时哪个实例找到该解（如 INVJ 0–3） |
| `sol_id`    | 本次求解中该解在解集中的登记序号        |
| `error`     | 末端位姿误差                  |
| `q_init`    | 初始关节角（弧度）               |
| `q_sol`     | 求出的关节角（弧度）              |


KDL / TRAC-IK 的 CSV 列更简单：`idx,time,q_init,q_sol`。

### 模型（`urdf/`）

按厂商分子目录存放测试用 URDF，例如：

- `denso/vs050/`、`ur/ur5/`、`franka/panda.urdf`、`kuka/iiwa/` 等

`benchmark.cpp` 中 `robot_id` 与模型对应关系（0–14）：HSR → VS050 → xArm6 → IRB140 → Jaco → UR5 → UR10 → Gen3 → iiwa → xArm7 → Panda → Sawyer → Yumi → Fetch → Tiago。

### 包配置


| 文件               | 作用                                                       |
| ---------------- | -------------------------------------------------------- |
| `CMakeLists.txt` | 声明依赖（coiks、trac_ik、KDL、Pinocchio、CasADi、nlopt 等），编译两个节点。 |
| `package.xml`    | ROS 包名、许可证与运行/编译依赖。                                      |




## 终端输出

运行时初始化、读取文件、汇总统计、结果路径等提示为**中文**，例如：

```text
[INFO] 实验结果目录: .../output/003
[INFO] 结果写入: .../output/003/coiks_invj_multi_r1.csv
汇总:
coiks_invj_multi 找到 10000 个解（100%），...
[INFO] 基准测试完成。结果目录: .../output/003
```

第三方库（如 KDL 关于 root link inertia）的英文警告可能仍会出现，可忽略。

## 测试机器人

覆盖 15 种商业机器人，涵盖多种运动学构型：


| robot_id | 机器人型号    | 厂商                 | 自由度 | 类型    |
| -------- | -------- | ------------------ | --- | ----- |
| 0        | HSR      | Toyota Motor Corp. | 5   | 服务机器人 |
| 1        | VS050    | DENSO Robotics     | 6   | 工业机器人 |
| 2        | xArm6    | UFactory           | 6   | 协作机器人 |
| 3        | IRB140   | ABB                | 6   | 工业机器人 |
| 4        | Jaco     | Kinova             | 6   | 辅助机器人 |
| 5        | UR-5     | Universal Robots   | 6   | 协作机器人 |
| 6        | UR-10    | Universal Robots   | 6   | 协作机器人 |
| 7        | Gen3     | Kinova Robotics    | 7   | 研究平台  |
| 8        | LBR iiwa | KUKA               | 7   | 协作机器人 |
| 9        | xArm7    | UFactory           | 7   | 协作机器人 |
| 10       | Panda    | Franka Emika       | 7   | 研究平台  |
| 11       | Sawyer   | Rethink Robotics   | 7   | 协作机器人 |
| 12       | Yumi     | ABB                | 7   | 双臂机器人 |
| 13       | Fetch    | Fetch Robotics     | 8   | 移动操作臂 |
| 14       | Tiago    | PAL Robotics       | 8   | 服务机器人 |


在 `coiks_benchmark.launch` / `coiks_benchmark_path.launch` 中设置 `robot_id`（0–14）即可切换上述机器人。

## 对比求解器

- CoIKS（全部变体，由 `ik_solver` 指定）
- TracIK（`tracik_enable`）
- Orocos KDL（`kdl_enable`）
- INVJ-RR（可作为 CoIKS 单求解器变体）

