###############################################################################
#                         ROS1 Configuration                                  #
###############################################################################

# specify ROS package dependencies
set (ROS1_PACKAGE_BUILD_DEPS
  grid_map_core
  grid_map_msgs
  grid_map_ros
  pcl_conversions
  sensor_msgs
)
set (ROS1_PACKAGE_DEPS
  ${ROS1_PACKAGE_BUILD_DEPS}
)

# find catkin and required dependencies
set (CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
find_package (catkin REQUIRED COMPONENTS ${ROS1_PACKAGE_BUILD_DEPS})

# configure ROS1 package metadata
catkin_package (
  INCLUDE_DIRS ../include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${ROS1_PACKAGE_DEPS}
  DEPENDS EIGEN PCL OpenVDB
)

###############################################################################
#                              ROS Targets                                    #
###############################################################################

set (OVM_ROS1_LIBRARY_DEPS
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# build ROS1VoxelCloud library
add_library (ros1_voxel_cloud ros1_voxel_cloud.cpp)
target_include_directories (ros1_voxel_cloud PUBLIC ${OVM_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries (ros1_voxel_cloud PUBLIC ${OVM_ROS1_LIBRARY_DEPS})
target_compile_options (ros1_voxel_cloud PUBLIC "$<$<COMPILE_LANGUAGE:CXX>:-Werror;-Wall;-fconcepts>")

###############################################################################
#                                INSTALL                                      #
###############################################################################

# @TODO