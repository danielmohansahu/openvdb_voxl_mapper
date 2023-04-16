# Development Docker image for openvdb_voxl_aggregator
# 
# This image starts with a basic CUDA image and layers
# ROS and OpenVDB (and NanoVDB) on top.

FROM nvidia/cuda:11.7.1-devel-ubuntu22.04 AS cuda-base

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
      wget \
      software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# install OpenVDB dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      cmake \
      libblosc-dev \
      libjemalloc-dev \
      libglfw3-dev \
      freeglut3-dev \
      libeigen3-dev \
      libpcl-dev \
      libboost-dev \
      libtbb-dev \
      libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

# clone and build openvdb
WORKDIR /opt
RUN git clone https://github.com/AcademySoftwareFoundation/openvdb.git -b v10.0.1
RUN mkdir openvdb/build \
    && cd openvdb/build \
    && cmake -DOPENVDB_BUILD_NANOVDB=ON \
             -DOPENVDB_BUILD_VDB_VIEW=ON \
             -DOPENVDB_BUILD_VDB_RENDER=ON \
             -DNANOVDB_BUILD_TOOLS=ON \
             -DNANOVDB_BUILD_BENCHMARK=ON \
             -DNANOVDB_USE_CUDA=ON \
             -DNANOVDB_USE_OPENVDB=ON \
             -DNANOVDB_BUILD_EXAMPLES=ON \
             -DOPENVDB_BUILD_UNITTESTS=ON \
             -DNANOVDB_BUILD_UNITTESTS=ON \
             .. \
    && make install -j12

# drop into expected workspace
WORKDIR /workspace
CMD bash
