# openvdb_voxel_mapper

Voxel mapping library based on the [OpenVDB](https://www.openvdb.org/) data structure.

OVM (OpenVDB Voxel Mapper) performs point cloud aggregation and processing to generate 2.5D maps via geometric and / or semantic operations. A [ROS 1](https://www.ros.org/) wrapper is provided which allows for direct subscription to `sensor_msgs::PointCloud2` topics and publication of [grid_map_msgs::GridMap](https://github.com/ANYbotics/grid_map) maps, with frame transformation handled by [tf2](http://wiki.ros.org/tf2).

## Prerequisites

The following dependency matrix outlines the core package dependencies and their support across operating system. Note that Ubuntu 22.04 [does not officially support ROS1](https://www.ros.org/reps/rep-0003.html#noetic-ninjemys-may-2020-may-2025), so ROS bindings aren't available for that OS.

Dependency | Ubuntu 20.04 | Ubuntu 22.04 | Notes
--- | --- | --- | ---
[OpenVDB](https://www.openvdb.org/) | Required | Required | Core data structure.
[NanoVDB](https://developer.nvidia.com/nanovdb) | Required | Required | GPU support for core data structure.
[PCL](https://pointclouds.org/) | Required | Required | Internal point cloud data representation and utilities.
[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) | Required | Required | Internal map data representation.
[ROS 1](https://www.ros.org/) | Optional | Unsupported | Bindings and examples.

The recommended workflow leverages [Containerization via Docker](https://www.docker.com/) to perform dependency management. This requires [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) to support NVIDIA GPUs within a Docker container. We also use [rocker](https://github.com/osrf/rocker)

Assuming all `docker` dependencies are installed you can build an Image and instantiate a Container via the provided bash script:

```bash
./build_and_run.sh
```

## Build

Build instructions vary based on whether or not you're using the ROS ecosystem. Each instruction set assumes you've entered a Docker container via the recommended development process.

Build for ROS | Build Core
--- | ---
`catkin build` | `cd /workspace/src/openvdb_voxel_mapper/`<br>`mkdir -p build && cd build`<br>`cmake .. -DCMAKE_BUILD_TYPE=Release -DOVM_BUILD_ROS1=OFF && make`

## ROS Runtime Examples

Several example scripts, nodes, and launch files are provided to demonstrate ROS worflows. All scripts assume you've sourced your workspace via `source devel/setup.bash`.

Example | Command
--- | ---
Offline Bagged Cloud Aggregation | `rosrun openvdb_voxel_mapper aggregate_bag {BAGFILE}`
Online Time-Limited Ground Plane Extraction | `roslaunch openvdb_voxel_mapper receding_ground_extraction_playback.launch bag:={BAGFILE} fixed_frame:={FRAME_ID} cloud_topic:={CLOUD_TOPIC}`

## Examples

The following are GIFs collected during execution of the Ground Plane Extraction ROS example. Note that the choppiness is generally due to RVIZ processing struggles, not node throughput!

Dataset | Recording | Recording
--- | --- | ---
[Rellis3D](https://arxiv.org/abs/2011.12954) | ![Ground Plane and Raw PC](docs/rellis_3d_playback_map.gif) | ![Ground Plane and Aggregated PC](docs/rellis_3d_playback.gif)
[DARPA SubT](https://link.springer.com/chapter/10.1007/978-3-030-71151-1_35) | ![Ground Plane and Raw PC](docs/subt_hallway_playback_map.gif) | ![Ground Plane and Aggregated PC](docs/subt_hallway_playback.gif)


## References:

The following papers / codebases were instrumental to development:
 - https://www.researchgate.net/publication/355133596_VDB-Mapping_A_High_Resolution_and_Real-Time_Capable_3D_Mapping_Framework_for_Versatile_Mobile_Robots
 - https://arxiv.org/abs/2211.04067
 - https://arxiv.org/abs/2105.04419
 - https://www.mdpi.com/2072-4292/14/21/5463
   - https://github.com/ViWalkerDev/NanoMap
 - https://www.researchgate.net/publication/340864490_Spatio-temporal_voxel_layer_A_view_on_robot_perception_for_the_dynamic_world
   - https://github.com/SteveMacenski/spatio_temporal_voxel_layer

Datasets used for examples:
 - https://arxiv.org/abs/2011.12954
 - https://link.springer.com/chapter/10.1007/978-3-030-71151-1_35
 - http://www.semantic-kitti.org/
