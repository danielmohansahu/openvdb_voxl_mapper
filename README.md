** PROJECT IN DEVELOPMENT **

# openvdb_voxel_mapper

A VOXEL mapper based on the OpenVDB data structure.

## Source Build Instructions

#### Prerequisites

TODO

#### Development Environment

The following convenience bash script simply builds the Docker Image and drops the user into an interactive Container via `rocker`.

```bash
./build_and_run.sh
```

#### Build

To build the codebase (assuming you're in the `/workspace` folder of the Docker Container):

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

## Examples / Runtime Usage

TODO

## References:

The following papers / codebases were instrumental to development:
 - https://www.researchgate.net/publication/355133596_VDB-Mapping_A_High_Resolution_and_Real-Time_Capable_3D_Mapping_Framework_for_Versatile_Mobile_Robots
 - https://arxiv.org/abs/2211.04067
 - https://arxiv.org/abs/2105.04419
 - https://www.mdpi.com/2072-4292/14/21/5463
   - https://github.com/ViWalkerDev/NanoMap
 - https://www.researchgate.net/publication/340864490_Spatio-temporal_voxel_layer_A_view_on_robot_perception_for_the_dynamic_world
   - https://github.com/SteveMacenski/spatio_temporal_voxel_layer
