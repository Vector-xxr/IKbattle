#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

export CMAKE_PREFIX_PATH="/usr/local:/opt/ros/noetic${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export CMAKE_MODULE_PATH="/usr/local/share/cmake/Modules${CMAKE_MODULE_PATH:+:${CMAKE_MODULE_PATH}}"
export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

# Host repo is bind-mounted at /coiks. Wire catkin packages into the workspace.
WS="${CATKIN_WS:-/root/catkin_ws}"
SRC="${WS}/src"
mkdir -p "${SRC}"

link_pkg() {
  local name="$1"
  local target="/coiks/${name}"
  local link="${SRC}/${name}"
  if [[ -d "${target}" ]]; then
    ln -sfn "${target}" "${link}"
  fi
}

link_pkg "coiks"
link_pkg "coiks_benchmark"

# Prefer system ros-noetic-trac-ik; ignore empty submodule dir if present.
if [[ -d /coiks/trac_ik_lib && ! -f /coiks/trac_ik_lib/package.xml ]]; then
  touch /coiks/trac_ik_lib/CATKIN_IGNORE 2>/dev/null || true
fi

if [[ -f "${WS}/devel/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${WS}/devel/setup.bash"
fi

exec "$@"
