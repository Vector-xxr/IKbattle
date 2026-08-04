# CoIKS-Benchmark

用于在多种商业机械臂上对比 CoIKS、TracIK（及可选 KDL）的**路径跟踪**基准测试套件。

## 概述

本包以连续末端轨迹（`path_robot_*.csv`）评估 IK 求解器：

- 完整 6D 位姿约束（位置 + 姿态）
- 逐点求 IK，统计成功率与耗时
- **终端输出为中文**（求解器名等标识仍为英文）
- **结果 CSV** 写入本包 `output/` 下按实验编号递增的子目录

可视化可用 Pinocchio + MeshCat 回放同一条 path（`--mode csv`）。

## 目录与代码说明

```
coiks_benchmark/
├── CMakeLists.txt              # catkin 编译配置，生成 coiks_benchmark_path
├── package.xml                 # ROS 包元数据与依赖声明
├── README.md
├── src/
│   ├── benchmark_path.cpp      # 路径跟踪 IK 对比主程序
│   └── experiment_output.hpp   # 自动创建 output/001、002… 并拼 CSV 路径
├── scripts/
│   ├── viz_urdf.py             # Pinocchio + MeshCat 可视化 path / neutral
│   ├── convert_vive_to_path.py # OpenArm：vive.csv → path_robot_15.csv
│   └── requirements-viz.txt    # 可视化脚本 Python 依赖
├── launch/
│   └── coiks_benchmark_path.launch  # 启动路径跟踪测试（含 OpenArm robot_id=15）
├── data/                       # path / qinit / targets CSV（见下）
├── output/                     # 实验结果；每次运行新建 001/、002/…
└── urdf/                       # 各机器人 URDF 模型（含 openarm/ 与 openarm_description/）
```



### 源码（`src/`）


| 文件                      | 作用                                                                 |
| ----------------------- | ------------------------------------------------------------------ |
| `benchmark_path.cpp`    | 路径跟踪基准。读取 `path_robot_*.csv`，对比 CoIKS / TRAC-IK / KDL。编译产物：`coiks_benchmark_path`。 |
| `experiment_output.hpp` | 内部工具：在 `output/` 下创建下一个实验目录（`001`、`002`…），生成结果文件路径。 |



### Launch（`launch/`）


| 文件                            | 作用                                                                                     |
| ----------------------------- | -------------------------------------------------------------------------------------- |
| `coiks_benchmark_path.launch` | 配置并启动路径跟踪测试。可设 `robot_id`（0–15，含 OpenArm）、`ik_solver`、时限/误差、是否启用 TRAC-IK/KDL 等。 |



### URDF 可视化（`scripts/`）

用 **Pinocchio + MeshCat** 回放 path CSV（与 C++ 解耦）。脚本会把 URDF 里旧的绝对 mesh 路径改写到本仓库 `urdf/`。

| `--mode` | 数据 | 行为 |
|----------|------|------|
| `csv` | `data/path_robot_<robot_id>.csv` | 沿路径点连续运动（简易 IK，种子优先 `qinit[0]`） |
| `neutral` | 无 | 中性位静态 |

参数：`--index` 起始下标；`--count` 连续条数（不写则从 index 播到文件末尾）；播放节奏**仅** `--fps`（默认 33，约 0.03s/帧）。

```bash
cd coiks_benchmark
pip install -r scripts/requirements-viz.txt   # 首次：meshcat + pin

python3 scripts/viz_urdf.py --robot_id 7 --mode csv --index 0 --fps 33
python3 scripts/viz_urdf.py --robot_id 15 --mode csv --index 0 --fps 120
python3 scripts/viz_urdf.py --robot_id 1 --mode neutral
```

说明：Docker 镜像侧重 C++ Pinocchio，不一定自带 Python 绑定；可视化建议在宿主机运行。

常用启动：

```bash
roslaunch coiks_benchmark coiks_benchmark_path.launch
```

通过改 launch 中的 `ik_solver` 切换 **CoIKS** 变体，例如：`coiks_invj_multi`、`coiks_nlo`、`coiks_qp`、`coiks_invj_nlo` 等。  
`tracik_enable` / `kdl_enable` 控制是否同时跑对比基线 TRAC-IK、Orocos KDL。

### 数据（`data/`）


| 文件模式                                           | 作用                                    |
| ---------------------------------------------- | ------------------------------------- |
| `path_robot_<id>.csv`                          | 路径跟踪连续目标：**仅末端位姿** `x,y,z,qx,qy,qz,qw`（无关节角） |
| `qinit_robot_<name>_random_10000samples.csv`   | path 初始种子（按 `point_id` 取一行）；仓库内仍保留完整文件 |
| `targets_robot_<name>_random_10000samples.csv` | 历史数据，仍保留于仓库；当前无随机位姿基准程序读取 |
| `qinit_robot_openarm_random_10000samples.csv`  | OpenArm path 种子（当前 1 行、右臂 7 轴全 0） |




### 结果（`output/`）

每次完整 `roslaunch` 运行会在 `output/` 下新建编号目录，同一次实验的所有求解器结果放在同一目录：

```text
coiks_benchmark/output/
├── 001/
│   ├── kdl_path_r15.csv           # 若开启 kdl_enable
│   ├── trac_ik_path_r15.csv       # 若开启 tracik_enable
│   └── coiks_qp_path_r15.csv      # 名称随 ik_solver 变化
├── 002/
│   └── ...
└── .gitkeep
```

- **目录编号**：`001`、`002`、`003`…，按已有目录自动递增。
- **文件名**：`{求解器名}_path_r{robot_id}.csv`
  - 求解器名：`kdl` / `trac_ik` / `coiks_invj_multi` 等（来自 `get_solver_name()`）
  - `r{robot_id}`：对应 launch 中的机器人编号（如 `r15` = OpenArm）
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

- `denso/vs050/`、`ur/ur5/`、`franka/panda.urdf`、`kuka/iiwa/`、`openarm/openarm_right.urdf` 等

`benchmark_path` / `viz_urdf.py` 中 `robot_id`（0–15）：HSR → VS050 → xArm6 → IRB140 → Jaco → UR5 → UR10 → Gen3 → iiwa → xArm7 → Panda → Sawyer → Yumi → Fetch → Tiago → **OpenArm**。

### 包配置


| 文件               | 作用                                                       |
| ---------------- | -------------------------------------------------------- |
| `CMakeLists.txt` | 声明依赖（coiks、trac_ik、KDL、Pinocchio、CasADi、nlopt 等），编译 `coiks_benchmark_path`。 |
| `package.xml`    | ROS 包名、许可证与运行/编译依赖。                                      |




## 终端输出

运行时初始化、读取文件、汇总统计、结果路径等提示为**中文**，例如：

```text
[INFO] 实验结果目录: .../output/003
[INFO] 结果写入: .../output/003/coiks_qp_path_r15.csv
[INFO] 基准测试完成。结果目录: .../output/003
```

第三方库（如 KDL 关于 root link inertia）的英文警告可能仍会出现，可忽略。

## 测试机器人

覆盖 15 种商业机器人 + OpenArm，涵盖多种运动学构型：


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
| 15       | OpenArm  | OpenArm            | 7   | 右臂专用 path 模型 |


在 `coiks_benchmark_path.launch` / `viz_urdf.py` 中设置 `robot_id`（0–15）。

## OpenArm 接入（robot_id=15）

OpenArm **右臂专用** URDF（7 DoF）与右臂末端轨迹已接入 path 可视化 / 路径跟踪基准。完整上半身模型保留为 `urdf/openarm/openarm.urdf`（可视化对照用），**benchmark / viz 默认用** `openarm_right.urdf`。

### 范围与约定

| 项 | 说明 |
| --- | --- |
| `robot_id` | **15**（`viz_urdf.py`、`benchmark_path.cpp`、`coiks_benchmark_path.launch`） |
| 模型 | `openarm/openarm_right.urdf`：躯干 + **右臂 7 轴** + `link_ee`（无左臂、无夹爪关节） |
| EE | `link_ee` ← fixed 挂在 `openarm_right_hand_tcp`；链 `base_link`→`link_ee` |
| path CSV | **只有末端位姿**，无关节角；关节角由 IK 在线求解 |

### 文件布局

```text
urdf/openarm/openarm.urdf              # 完整上半身（双臂，保留）
urdf/openarm/openarm_right.urdf        # 右臂专用 7 DoF（path / CoIKS 默认）
urdf/openarm_description/assets/...    # package://openarm_description/... 所需 mesh
data/path_robot_15.csv                 # 右臂末端路径（x,y,z,qx,qy,qz,qw）
data/qinit_robot_openarm_random_10000samples.csv  # path 种子：problem_id + q1..q7=0
scripts/convert_vive_to_path.py        # vive 相对轨迹 → path_robot_15.csv
```

URDF：由完整 `openarm.urdf` 抽取躯干 + 右臂；根为 `base_link`；`link_ee` 挂在右臂 TCP。mesh 仍用 `package://openarm_description/...`。

### 轨迹数据（vive → path）

来源一般为 all_telop 录制的 `sensor/vive.csv`（相对首次锁定 T0 的右臂期望 TCP）。转换约定：

- **T0** = 右臂 TCP（`link_ee`）在 **q=0（手臂垂直）** 下的 FK
- 与 `TcpRelativeCsvWriter` 一致：`p_abs = p0 + p_rel`，`R_abs = R0 · R_rel`（位置**不**乘 `R0`）
- 输出列顺序改为基准格式 `x,y,z,qx,qy,qz,qw`

重新生成示例：

```bash
cd coiks_benchmark
python3 scripts/convert_vive_to_path.py \
  --vive /path/to/episode/sensor/vive.csv \
  --urdf urdf/openarm/openarm_right.urdf \
  --out data/path_robot_15.csv
```

### 可视化时关节角如何得到

`path_robot_15.csv` 不含 `q`。`viz_urdf.py --mode csv` 对每个目标位姿做阻尼伪逆 IK：

1. 种子：优先 `qinit[0]`（7 轴，与右臂模型 `nq` 一致）
2. 目标：CSV 的完整 6D 位姿 → `link_ee`
3. 上一帧成功解作为下一帧初值（连续跟踪）

常用命令：

```bash
cd coiks_benchmark
pip install -r scripts/requirements-viz.txt   # 首次
python3 scripts/viz_urdf.py --robot_id 15 --mode neutral
python3 scripts/viz_urdf.py --robot_id 15 --mode csv --index 0 --fps 120
# 不写 --count：从 index 播到 CSV 末尾；节奏仅用 --fps
```

### 路径跟踪基准

```bash
# launch 中 robot_id 已默认 15；或自行修改
roslaunch coiks_benchmark coiks_benchmark_path.launch
```

`benchmark_path` 读 `path_robot_15.csv` + 上述 7 轴 qinit，用 COIKS（及可选 TRAC-IK / KDL）对 `base_link`→`link_ee` 逐点求 IK；结果写入 `output/<编号>/`。

## 对比求解器

- CoIKS（全部变体，由 `ik_solver` 指定）
- TracIK（`tracik_enable`）
- Orocos KDL（`kdl_enable`）
- INVJ-RR（可作为 CoIKS 单求解器变体）
