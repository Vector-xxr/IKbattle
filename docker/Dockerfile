# CoIKS runtime/build image: Ubuntu 20.04 + ROS Noetic
# Host OS can be Ubuntu 22.04; only the container needs 20.04.
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    CATKIN_WS=/root/catkin_ws \
    CMAKE_PREFIX_PATH=/usr/local:/opt/ros/noetic

SHELL ["/bin/bash", "-c"]

# -----------------------------------------------------------------------------
# System + ROS packages
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    pkg-config \
    python3-pip \
    python3-catkin-tools \
    libeigen3-dev \
    libboost-all-dev \
    liburdfdom-dev \
    libassimp-dev \
    liboctomap-dev \
    libconsole-bridge-dev \
    coinor-libipopt-dev \
    libnlopt-dev \
    libblas-dev \
    liblapack-dev \
    gfortran \
    liborocos-kdl-dev \
    ros-noetic-eigen-conversions \
    ros-noetic-kdl-parser \
    ros-noetic-trac-ik \
    ros-noetic-urdf \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# CasADi (with IPOPT + OSQP backends used by NLO / QP solvers)
# -----------------------------------------------------------------------------
ARG CASADI_VERSION=3.6.5
WORKDIR /tmp
RUN git clone --depth 1 --branch ${CASADI_VERSION} https://github.com/casadi/casadi.git \
    && mkdir -p casadi/build && cd casadi/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DWITH_IPOPT=ON \
        -DWITH_OSQP=ON \
        -DWITH_BUILD_OSQP=ON \
        -DWITH_THREAD=ON \
        -DWITH_PYTHON=OFF \
        -DWITH_PYTHON3=OFF \
        -DWITH_MATLAB=OFF \
        -DWITH_OCTAVE=OFF \
        -DWITH_EXAMPLES=OFF \
        -DWITH_DOC=OFF \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && cd / && rm -rf /tmp/casadi

# -----------------------------------------------------------------------------
# Pinocchio with CasADi autodiff support (needed by coiks headers)
# -----------------------------------------------------------------------------
ARG PINOCCHIO_VERSION=v2.6.21
WORKDIR /tmp
RUN git clone --depth 1 --branch ${PINOCCHIO_VERSION} --recursive \
        https://github.com/stack-of-tasks/pinocchio.git \
    && mkdir -p pinocchio/build && cd pinocchio/build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_PYTHON_INTERFACE=OFF \
        -DBUILD_WITH_CASADI_SUPPORT=ON \
        -DBUILD_WITH_URDF_SUPPORT=ON \
        -DBUILD_TESTING=OFF \
        -DINSTALL_DOCUMENTATION=OFF \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && cd / && rm -rf /tmp/pinocchio

# CoIKS CMake uses find_package(CASADI); provide a thin compatibility module.
COPY FindCASADI.cmake /usr/local/share/cmake/Modules/FindCASADI.cmake
ENV CMAKE_MODULE_PATH=/usr/local/share/cmake/Modules

# -----------------------------------------------------------------------------
# Catkin workspace layout
# Mount host repo packages into src/ at runtime (see docker-compose.yml).
# -----------------------------------------------------------------------------
RUN mkdir -p ${CATKIN_WS}/src \
    && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "source ${CATKIN_WS}/devel/setup.bash 2>/dev/null || true" >> /root/.bashrc

WORKDIR ${CATKIN_WS}
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
