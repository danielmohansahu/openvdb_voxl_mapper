###############################################################################
#                         ROS1 Configuration                                  #
###############################################################################

# specify ROS package dependencies
set (ROS1_PACKAGE_BUILD_DEPS
  grid_map_core
  grid_map_msgs
  grid_map_ros
  pcl_conversions
  pcl_ros
  rosbag
  sensor_msgs
  tf2_ros
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
add_library (ros1_voxel_cloud src/ros1_voxel_cloud.cpp)
target_include_directories (ros1_voxel_cloud PUBLIC ${OVM_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries (ros1_voxel_cloud PUBLIC ${OVM_ROS1_LIBRARY_DEPS})
target_compile_options (ros1_voxel_cloud PUBLIC "$<$<COMPILE_LANGUAGE:CXX>:-Werror;-Wall;-fconcepts>")

# example / utility executable to support aggregating clouds in a bag file
add_executable (aggregate_bag utilities/aggregate_bag.cpp)
target_include_directories (aggregate_bag PRIVATE ${OVM_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries (aggregate_bag ${OVM_ROS1_LIBRARY_DEPS})
target_compile_options (aggregate_bag PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-Werror;-Wall;-fconcepts>")

# example / node executable to demonstrate runtime processing
add_executable (receding_ground_extraction_node nodes/receding_ground_extraction_node.cpp)
target_include_directories (receding_ground_extraction_node PRIVATE ${OVM_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries (receding_ground_extraction_node ${OVM_ROS1_LIBRARY_DEPS})
target_compile_options (receding_ground_extraction_node PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-Werror;-Wall;-fconcepts>")

###############################################################################
#                                INSTALL                                      #
###############################################################################

# install build targets
install (TARGETS ros1_voxel_cloud aggregate_bag receding_ground_extraction_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# install configuration / runtime directories
install (DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
