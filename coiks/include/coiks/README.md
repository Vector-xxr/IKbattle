# `coiks/` 头文件说明

本目录是 CoIKS **调度与并发层** 的公共接口：定义求解器抽象、配置、解集合，以及对外主类 `COIKS`。具体 INVJ / QP / NLO 算法实现分别在 `invj/`、`qp/`、`nlo/`。

| 文件 | 作用 |
|------|------|
| [`coiks.hpp`](coiks.hpp) | 主类 `COIKS`：加载 URDF、调度单/多求解器、并发求解与选优 |
| [`ik_solver.hpp`](ik_solver.hpp) | 对外统一 IK 接口 `IkSolver`（虚函数） |
| [`solver_base.hpp`](solver_base.hpp) | 底层求解器公共基类 `SolverBase` 与常用类型别名 |
| [`solver_options.hpp`](solver_options.hpp) | 运行参数结构体 `SolverOptions`（INVJ/QP/NLO/选优模式等） |
| [`solution_set.hpp`](solution_set.hpp) | 多解集合 `IKSolutionSet`：存解、指标，按准则取最优 |

依赖关系概览：

```text
IkSolver  <---  COIKS（实现并发调度）
                  |
                  +-- SolverOptions
                  +-- IKSolutionSet
                  +-- INVJ / QP / NLO（各自继承 SolverBase）
```

---

## `coiks.hpp`

声明命名空间 `coiks` 下的主类 **`COIKS`**（继承 `IkSolver`）。

职责：

- 用 Pinocchio 加载 URDF，建立模型与末端 frame
- 按 `ik_solver` 名称初始化单求解器或组合（如 `coiks_invj_multi`、`coiks_invj_qp`、`coiks_all`）
- **`IkSolve`**：对外求 IK，并填入 `bestSolution`
- **`concurrentIkSolve`**：多线程并行跑多个求解器，先到/多解再选优
- FK / 位姿查询：`updateFK`、`getEEPose`、`getFramePose`
- 统计：`printStatistics`

内部持有 `INVJ` / `NLO` / `QP` 求解器指针、多 INVJ 实例、线程与互斥锁；具体实现见 `src/coiks.cpp`。

---

## `ik_solver.hpp`

轻量抽象基类 **`IkSolver`**，统一「能求 IK 的对象」对外接口：

- `IkSolve(q_init, x_des, q_out)`
- `IkSolve(..., Solution &solution)`（带解元数据）
- `get_n_joints()` / `get_solver_name()` / `printStatistics()`

`COIKS` 以及部分包装器通过该接口被上层（如 benchmark）调用。

---

## `solver_base.hpp`

命名空间 `coiks` 内底层算法的公共基类 **`SolverBase`**：

- 共用成员：Pinocchio 模型 `mdl_`、末端 id、目标位姿、关节数、时间/误差上限、中止与成功标志
- 虚接口 `IkSolve`
- 类型别名：`Vector6d`、`Twist`，以及角度单位常量

INVJ / QP / NLO 的具体求解类在此基础上扩展。

---

## `solver_options.hpp`

配置结构体 **`SolverOptions`**，由 ROS param / 启动文件等填入，贯穿所有求解器，主要包括：

| 分组 | 典型字段 |
|------|----------|
| 通用 | `error_type_`、`solve_mode_`（speed / distance / manipulability） |
| INVJ | `pinv_method_`、`invj_Ke1_`、限位与种子策略、多实例数量与停滞阈值 |
| NLO | `nlo_linear_solver_`、`cost_coeff_`、`nlo_warm_start_`、并发相关 |
| QP | `qp_K_t1_`、`qp_Kd_p1_`、`qp_Kw_p1_`、`qp_warm_start_` 等 |
| 日志 | `verb_level_`、`time_stats_`、`other_opts_` |

不含算法逻辑，只做参数容器。

---

## `solution_set.hpp`

线程安全的解集合 **`IKSolutionSet`**：

- **`Solution`**：关节构型、指标 map、耗时、`solver_id` / `solution_id`
- 增删解、登记指标名（如 `error`、`distance`、`manipulability`）
- **`getBestSolution`**：按指定指标最大化或最小化选优（`OptimizationGoal`）

并发模式下多个求解器往同一集合写解；上层再按 `solve_mode` 或误差准则取最优。

---

## 相关文档

- 三类算法原理：[../README.md](../README.md)
- 包级说明：[../../README.md](../../README.md)
