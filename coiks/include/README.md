# CoIKS 逆运动学算法说明

本文整理本仓库三类底层 IK 求解器（INVJ / QP / NLO）的原理与实现要点，对应目录：

| 算法 | 目录 | 核心文件 |
|------|------|----------|
| INVJ-RR | `invj/` | `invj_solvers.hpp` |
| QP | `qp/` | `qp_solvers.hpp` |
| NLO | `nlo/` | `nlo_solvers.hpp` |

调度与并发组合见上层 `coiks/`（`coiks.hpp` / `src/coiks.cpp`）。

---

## 1. 三类方法总览

三者都在**迭代**中缩小末端误差，差别在于「每一步怎么算关节更新」以及「限位怎么进公式」。

| 维度 | INVJ | QP | NLO |
|------|------|-----|-----|
| 层级 | 速度级 | 速度级 | 位置级（优化 `q_delta`） |
| 是否线性化 | 是（用雅可比 `J`） | 是（用 `J`） | 否（直接用 FK） |
| 核心计算 | 伪逆闭式解 | OSQP 凸二次规划 | IPOPT 非线性规划 |
| 限位 | 事后 clamping / random | 速度不等式硬约束 | 变量上下界 |
| 典型耗时 | 微秒级 | 毫秒级 | 数十毫秒（可 warm start） |

直观类比：

- **INVJ**：蒙眼短跑——朝误差方向冲，撞限再切或重开。
- **QP**：看车道线的车——每步速度都先保证不压限位。
- **NLO**：看整张地图——用真实 FK 在限位围栏里找最优关节增量。

---

## 2. 误差、李代数与速度指令

### 2.1 速度级公共骨架

INVJ / QP 每步大致是：

1. FK 得到当前末端位姿 `B_X_EE`
2. 算误差向量 `err_ee`（6 维）
3. 期望笛卡尔速度 `v ≈ Kee * err_ee`
4. 用 `J` 把 `v` 映射到关节速度 `q_dot`（伪逆或 QP）
5. `q = integrate(q, q_dot * dt)`，再检查/施加限位

### 2.2 李代数只用在「算误差」

**在本仓库速度级算法里，`log6` / `log3` 的唯一直接用途是把位姿差变成可运算的 6 维向量 `err_ee`。**

它不参与：

- 雅可比 `J` 的构造
- 伪逆 / OSQP / IPOPT 内部求解
- `integrate` 更新本身（关节更新用 Pinocchio 的 `integrate`，那是另一处的李群运算）

为什么必须用 log？

- 齐次矩阵 `X_d`、`B_X_EE` **不能直接相减**（旋转部分相减没有物理意义）。
- `log` 把相对位姿映到切空间（李代数），得到  
  `[vx, vy, vz, wx, wy, wz]`。
- 之后 `v = Kee * err_ee` 只是普通数乘；再往后全是线性代数或数值优化。

`log6` vs `log3`：

- **log3（解耦）**：位置欧氏差 + 姿态轴角，平移/旋转指令相对独立。
- **log6（耦合）**：SE(3) 上的螺旋（测地）误差，平移与旋转揉在同一 6 维旋量里。

二者都只决定「误差向量的方向与尺度」，不改变后面「伪逆/优化」的数学形式。

### 2.3 log 可逆吗？

- **数学上可逆**：`log` 与 `exp` 互逆（Pinocchio 中如 `log6` / `exp6`）。
- **信息上**：只能还原**相对变换**（位姿差盒子），**不能**单凭误差向量还原原来的两个绝对位姿。  
  需要保留其中一个基准位姿，才能用 `exp(err)` 推出另一个。

类比：知道「温差 5°C」不能唯一推出昨今气温，必须再知道其中一个。

---

## 3. INVJ-RR（逆雅可比 + 随机重启）

### 3.1 核心公式

```text
q_dot = pinv(J) * (Kee * e)
q     = integrate(q, q_dot * dt)
```

- `e = err_ee`：由 `log3` / `log6` 得到
- `pinv(J)`：伪逆（代码可选 `svd` / `ldlt` / `cod` 等）
- 随机重启：停滞或越界时 `randomConfiguration`，跳出局部极小

### 3.2 限位细节（实现要点）

限位来自 URDF：`mdl_.lowerPositionLimit` / `upperPositionLimit`。

| 策略 `limiting_method` | 行为 | 注意 |
|------------------------|------|------|
| **clamping** | 积分后硬裁到 `[q_ll, q_ul]` | 简单安全；卡在边界时易出现「算了速度但位置不动」的虚拟积累 |
| **random** | 任一关节越界则整构型随机合法重置 | 仿真可用；实物上瞬移危险，不宜做实时伺服主策略 |

时序：先算 `q_dot` → `integrate` → **再**限位处理 → 再 FK/误差。

其它工程注意：

- `pin::integrate` 会对旋转角做归一化（wrap），可能与 URDF 非对称限位冲突，导致「数学等价角」触发越界判定。
- 本实现**没有**显式 `q_dot` 速度饱和；大误差时单步可能偏猛，实机建议加速度限幅并适当减小 `Kee`。

---

## 4. QP（二次规划，速度级 + 硬约束）

### 4.1 为什么需要 QP？

INVJ 限位是**事后**补救。QP 在算 `q_dot` 时就把「下一步不准超限」写进优化问题，减少撞墙后硬切/乱跳。

### 4.2 标准形式

```text
minimize   0.5 * x' * Q * x + p' * x
subject to C * x <= d
```

决策变量：

```text
x = [ q_dot ; w ]
```

- `q_dot`：关节速度  
- `w`：松弛变量——任务实在完不成时允许残留笛卡尔误差，避免硬拧坏限位/奇异

### 4.3 符号含义（目标函数）

期望笛卡尔速度：`v_des = Kee * e`。代价直觉上含三部分（权重由 `qp_K_t1_` / `qp_Kd_p1_` / `qp_Kw_p1_` 等调节）：

1. 跟踪：`|| J * q_dot - v_des ||^2`（可与松弛一起建模）  
2. 阻尼：`|| q_dot ||^2`（别乱动）  
3. 松弛惩罚：`|| w ||^2`（别轻易放弃任务）

展开平方项后出现 `q_dot' * (...) * q_dot` 等**二次型**，于是得到 `Q`、`p`：

- `Q`：二次代价（对称；含 `J' * J`、阻尼、松弛块等）  
- `p`：一次项（鼓励 `q_dot` 朝减小跟踪误差的方向）  
- 前面的 `0.5`：求导方便的惯例，不改变最优解

### 4.4 约束 `C * x <= d`（限位 → 速度界）

由下一步位置不超限：

```text
q_ll <= q + q_dot * dt <= q_ul
```

得到：

```text
q_dot  <= (q_ul - q) / dt
-q_dot <= (q - q_ll) / dt
```

写成 `C * x <= d` 时，`w` 列通常为 0：**松弛不放松物理限位**，只放松任务精度。

求解器：CasADi + **OSQP**（凸 QP，适合实时附近频段）。

### 4.5 为什么叫「二次规划」？

- **规划（Programming）**：在约束下做最优决策（数学优化，不是写代码）。  
- **二次（Quadratic）**：目标函数对决策变量最高为二次（来自 `||...||^2`），约束为线性不等式。  

二次凸目标像「开口向上的碗」，有清晰的全局最低点（在凸设定下），轨迹往往比硬切限位更平滑。

---

## 5. NLO（非线性优化，位置级）

### 5.1 核心思想

不再用 `J * q_dot ≈ v` 的一阶近似，而是优化关节增量 `q_delta`，直接用：

```text
FK(q_init + q_delta)
```

与目标位姿比较（位置欧氏距离 + `log3`/`log6` 姿态误差等），权重 `mu0/mu1/mu2`…。

形式概览：

```text
minimize  mu0 * ||p_err||^2 + mu1 * ||R_err||^2 + mu2 * ||q_delta||^2
subject to  q_ll <= q_init + q_delta <= q_ul
```

（限位常做成优化变量的 `lbx/ubx` 边界。）

### 5.2 求解与 warm start

- 求解器：CasADi + **IPOPT**  
- `IkSolve` 循环里可用上一步解作初值（warm start），连续跟踪时显著加速  

优点：建模最贴原始运动学，大范围/高精度更强。  
缺点：最慢；调参（容差、迭代上限、代价权重）更重；FK 非凸，仍可能有局部极小，但比纯速度线性化往往更稳。

---

## 6. 和 CoIKS 并发框架的关系

- **单求解器**：`coiks_invj` / `coiks_qp` / `coiks_nlo`  
- **并发组合**：如 `coiks_invj_multi`、`coiks_invj_qp`、`coiks_all`——多线程提高成功率与平均速度  
- **`solve_mode`**：`speed` / `distance` / `manipulability` 决定多解时如何选优  

实现细节注意：`IkSolve` 里写入 `bestSolution` 时，非 distance/manipulability 会走按 **`error` 最小**取优；与 `concurrentIkSolve` 在 `speed` 下取 `q_solutions_[0]` 可能不完全一致。读 CSV 的 `sol_id` 时需结合这一点理解。

---

## 7. 怎么选（实用）

| 需求 | 更合适 |
|------|--------|
| 最高频伺服、算力紧 | INVJ |
| 要硬限位、轨迹更稳、还能接受毫秒级 | QP |
| 大范围、高精度、可偏离线/中低频 | NLO |
| 要成功率/速度折中 | CoIKS 并发（如 `coiks_invj_multi`） |

---

## 8. 附录：基准测试中的 QP / NLO 运行参数注意

### 8.1 QP（path 基准）

Launch（`coiks_benchmark_path.launch`）与 `benchmark_path.cpp` 使用 ROS 参数名 **`hqp_*`**，
并写入 `SolverOptions`：

| Launch / ROS 参数 | `SolverOptions` 字段 | 含义 |
|-------------------|----------------------|------|
| `hqp_K_t1` | `qp_K_t1_` | 任务跟踪权重 |
| `hqp_Kr_t1` | `qp_Kr_t1_` | 笛卡尔速度残差系数 |
| `hqp_Kd_p1` | `qp_Kd_p1_` | 关节阻尼 / 正则 |
| `hqp_Kw_p1` | `qp_Kw_p1_` | 松弛惩罚 |
| `hqp_warm_start` | `qp_warm_start_` | OSQP warm start |

若误读成 `qp_*` 或漏赋 `qp_Kd_p1_`，日志里会出现 `K=1`、`Kd` 为垃圾值，成功率可到 0%。

### 8.2 NLO（IPOPT）

本仓库 Docker 常见 **IPOPT 3.11.x**，不支持 `ipopt.max_wall_time`（需 3.14+）。
NLO 初始化应使用：

```text
ipopt.max_cpu_time = max_time
```

否则 CasADi 会抛 `No such IPOPT option: max_wall_time` 并进程退出。

---

## 9. 延伸阅读

- 包级说明：[../README.md](../README.md)  
- 基准与 CSV 输出：[../../coiks_benchmark/README.md](../../coiks_benchmark/README.md)  
- Docker：[../../docker/README.md](../../docker/README.md)  

本文档对应源码实现口径；第三方库（Pinocchio / CasADi / OSQP / IPOPT）细节以其官方文档为准。
