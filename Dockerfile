# Development Docker image for openvdb_voxl_aggregator
# 
# This image starts with a basic CUDA image and layers
# ROS and OpenVDB (and NanoVDB) on top.

FROM nvidia/cuda:12.1.0-devel-ubuntu20.04 AS cuda-base

# install common utilities
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      vim \
      byobu \
      gdb \
      lsb-release \
      curl \
      git \
      cmake \
    && rm -rf /var/lib/apt/lists/*

# install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      ros-noetic-ros-base \
    && rm -rf /var/lib/apt/lists/*

# install OpenVDB dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      libboost-iostreams-dev \
      libtbb-dev \
      libblosc-dev \
    && rm -rf /var/lib/apt/lists/*

# clone and build openvdb
WORKDIR /opt
RUN git clone https://github.com/AcademySoftwareFoundation/openvdb.git -b v9.1.0
RUN mkdir openvdb/build \
    && cd openvdb/build \
    && cmake -DOPENVDB_BUILD_NANOVDB=ON \
             -DNANOVDB_BUILD_TOOLS=ON \
             -DNANOVDB_BUILD_EXAMPLES=ON \
             -DNANOVDB_BUILD_BENCHMARK=ON \
             # -DOPENVDB_BUILD_UNITTESTS=ON \
             # -DNANOVDB_BUILD_UNITTESTS=ON \
             .. \
    && make \
    && make install

# drop into expected workspace
WORKDIR /home/root/workspace
CMD bash
