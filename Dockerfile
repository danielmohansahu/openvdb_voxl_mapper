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
      wget \
      software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# add ROS and kitware PPAs
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 42D5A192B819C5DA
RUN apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# install ROS Noetic and latest cmake, as well as OpenVDB dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      ros-noetic-ros-base \
      cmake \
      libblosc-dev \
      libjemalloc-dev \
      libglfw3-dev \
      freeglut3-dev \
      libeigen3-dev \
      libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# install boost (default Ubuntu distribution is too low)
WORKDIR /tmp
RUN wget -nv https://boostorg.jfrog.io/artifactory/main/release/1.81.0/source/boost_1_81_0.tar.gz
RUN tar -xzf boost_1_81_0.tar.gz \
    && cd boost_1_81_0 \
    && ./bootstrap.sh \
    && ./b2 install

# install TBB (default Ubuntu distribution is too low)
RUN git clone https://github.com/oneapi-src/oneTBB.git -b v2021.8.0
RUN mkdir oneTBB/build \
    && cd oneTBB/build \
    && cmake -DTBB_TEST=OFF .. \
    && cmake --build . \
    && cmake --install .

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
             # -DOPENVDB_BUILD_UNITTESTS=ON \
             # -DNANOVDB_BUILD_UNITTESTS=ON \
             .. \
    && make install -j12

# drop into expected workspace
WORKDIR /workspace
CMD bash
