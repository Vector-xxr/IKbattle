# CoIKS Docker（Ubuntu 20.04 + ROS Noetic）

在宿主机（例如 Ubuntu 22.04）上通过 Docker 运行 CoIKS，无需本机安装 ROS1。

## 环境说明


| 项目          | 内容                                                     |
| ----------- | ------------------------------------------------------ |
| 基础镜像        | `ros:noetic-ros-base`（Ubuntu 20.04）                    |
| 镜像名         | `coiks:noetic-20.04`                                   |
| 容器名         | `coiks-noetic`                                         |
| 源码挂载        | 宿主机仓库 → 容器内 `/coiks`                                   |
| Catkin 工作空间 | `/root/catkin_ws`                                      |
| 预装依赖        | Pinocchio（含 CasADi）、CasADi（IPOPT + OSQP）、TRAC-IK、KDL 等 |


镜像只负责依赖；**CoIKS 源码需在容器内再执行一次** `catkin_make`。

## 目录结构

```
CoIKS/
├── docker-compose.yml
└── docker/
    ├── Dockerfile
    ├── entrypoint.sh      # 挂载源码到 catkin src，并 source ROS
    ├── FindCASADI.cmake   # 兼容 find_package(CASADI)
    ├── build_ws.sh        # 容器内一键编译
    └── README.md
```



## 快速开始

在**仓库根目录**执行：

```bash
cd /path/to/CoIKS

# 1. 构建镜像（首次较慢，需编译 CasADi / Pinocchio）
docker compose build

# 2. 启动容器
docker compose up -d

# 3. 进入容器并编译工作空间
docker compose exec coiks bash
bash /coiks/docker/build_ws.sh
```

编译成功后会出现：

```text
[完成] 工作空间编译成功。示例:
  roslaunch coiks_benchmark coiks_benchmark_path.launch
```

然后在**同一容器 shell**中：

```bash
source /root/catkin_ws/devel/setup.bash   # 每个新 shell 执行一次

roslaunch coiks_benchmark coiks_benchmark_path.launch
```

NLO 相关：默认线性求解器可能是 `ma57`（需 HSL）。无授权时可在 launch 中设置：

```xml
<param name="nlo_linear_solver" value="mumps" type="str"/>
```

（当前默认若为 `coiks_invj_multi` 等非 NLO 模式，可不设。）



## 常用命令

```bash
# 启动容器（后台运行）
docker compose up -d

# 进入已运行的容器
docker compose exec coiks bash

# 查看状态 / 日志
docker compose ps
docker compose logs -f coiks

# 修改 Dockerfile 后重新构建
docker compose build --no-cache   # 一般不需要 --no-cache
docker compose up -d --force-recreate
```

### 退出 shell ≠ 关闭容器

在容器内输入 `exit`（或 Ctrl+D）只是离开当前 bash，**容器仍在后台运行**（`docker compose up -d` 启动的）。

| 场景 | 做法 |
|------|------|
| 暂时不用 / 想省 CPU、内存 | 在**仓库根目录**执行 `docker compose stop` |
| 下次继续用 | `docker compose up -d`，再 `docker compose exec coiks bash` |
| 停止并删除容器（保留镜像与编译 volume） | `docker compose down` |

```bash
# 暂时不用了，关掉容器省资源（推荐）
docker compose stop

# 确认已停止
docker compose ps
```

`stop` 不会删镜像，也不会清掉 catkin 的 `build`/`devel` volume；再次 `up -d` 后一般不必重新 `docker compose build`，也不必重编（除非改过源码）。

每次新开一个 shell 进入容器后，都需要：

```bash
source /root/catkin_ws/devel/setup.bash
```

（`entrypoint.sh` 在登录时会尝试自动 source；若 `devel` 尚不存在则需先编译。）

## 工作方式

1. 宿主机改代码会立刻反映到容器 `/coiks`（bind mount）。
2. `entrypoint` 会把 `/coiks/coiks`、`/coiks/coiks_benchmark` 链到 `/root/catkin_ws/src/`。
3. `build` / `devel` / `logs` 使用 Docker volume，容器重启后编译产物仍在。
4. TRAC-IK 使用镜像内 `ros-noetic-trac-ik`，不必初始化空的 `trac_ik_lib` 子模块。



## 重新编译

改了源码或 CMake 后，在容器内：

```bash
bash /coiks/docker/build_ws.sh
# 或
cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_MODULE_PATH=/usr/local/share/cmake/Modules
source devel/setup.bash
```



## 常见问题



### 1. `devel/setup.bash: No such file or directory`

说明还没编译成功。先执行：

```bash
bash /coiks/docker/build_ws.sh
```

不要只 `source`；镜像构建 ≠ catkin 工作空间构建。

### 2. `Unable to locate package ros-noetic-orocos-kdl`

应使用系统包 `liborocos-kdl-dev`（当前 Dockerfile 已处理）。

### 3. CasADi 配置报 OSQP not found

需开启 `-DWITH_BUILD_OSQP=ON` 让 CasADi 自行编译 OSQP（当前 Dockerfile 已处理）。

### 4. 编译报 `std::make_unique` is only available from C++14

`coiks/CMakeLists.txt` 需使用 `-std=c++14`（仓库内已改为 C++14）。

### 5. NLO / IPOPT 与 `ma57`

默认线性求解器是 `ma57`，需要 HSL 授权。Docker 环境请使用：

```bash
rosparam set nlo_linear_solver mumps
```



### 6. Benchmark 结果 CSV 在哪

结果写在包内 `coiks_benchmark/output/001`、`002`…（宿主机与容器通过挂载共享）。详见 [coiks_benchmark/README.md](../coiks_benchmark/README.md)。

### 7. `exit` 之后还要不要关 Docker

`exit` 只退出容器内的 shell。暂时不用时请在仓库根目录执行：

```bash
docker compose stop
```

以释放资源；需要时再 `docker compose up -d`。

## 清理

```bash
# 停止并删除容器（保留镜像与 volume）
docker compose down

# 同时删除编译缓存 volume（下次需重新 catkin_make）
docker compose down -v

# 删除镜像
docker rmi coiks:noetic-20.04
```

