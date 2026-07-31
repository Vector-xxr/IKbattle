#!/usr/bin/env bash
# Run inside the container after `docker compose up -d` / `docker compose exec coiks bash`
set -euo pipefail

source /opt/ros/noetic/setup.bash
cd "${CATKIN_WS:-/root/catkin_ws}"

export CMAKE_PREFIX_PATH="/usr/local:/opt/ros/noetic${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export CMAKE_MODULE_PATH="/usr/local/share/cmake/Modules${CMAKE_MODULE_PATH:+:${CMAKE_MODULE_PATH}}"

# First-time / incremental build
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_MODULE_PATH=/usr/local/share/cmake/Modules

source devel/setup.bash
echo "[完成] 工作空间编译成功。示例:"
echo "  roslaunch coiks_benchmark coiks_benchmark.launch"
