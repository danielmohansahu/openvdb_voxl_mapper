###############################################################################
#                              ROS Targets                                    #
###############################################################################

# shared dependences / configuration for all ROS1 build targets
set (OVM_ROS1_DEPENDENCIES ${PROJECT_NAME} ${catkin_LIBRARIES})
set (OVM_ROS1_INCLUDE_DIRS ${OVM_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
set (OVM_ROS1_COMPILE_OPTIONS
  "$<$<COMPILE_LANGUAGE:CXX>:-Werror;-Wall;-fconcepts>"
  "$<$<COMPILE_LANGUAGE:CUDA>:SHELL:--compiler-options -Wall>"
)
set (OVM_ROS1_TARGET_PROPERTIES CXX_STANDARD 17 CXX_STANDARDS_REQUIRED TRUE)

# build ROS1VoxelCloud library
add_library (ros1_voxel_cloud src/ros1_voxel_cloud.cpp)
target_include_directories (ros1_voxel_cloud PUBLIC ${OVM_ROS1_INCLUDE_DIRS})
target_link_libraries (ros1_voxel_cloud PUBLIC ${OVM_ROS1_LIBRARY_DEPS})
target_compile_options (ros1_voxel_cloud PUBLIC ${OVM_ROS1_COMPILE_OPTIONS})
set_target_properties (ros1_voxel_cloud PROPERTIES ${OVM_ROS1_TARGET_PROPERTIES})

# example / utility executable to support aggregating clouds in a bag file
add_executable (aggregate_bag utilities/aggregate_bag.cpp)
target_include_directories (aggregate_bag PRIVATE ${OVM_ROS1_INCLUDE_DIRS})
target_link_libraries (aggregate_bag ${OVM_ROS1_DEPENDENCIES} ros1_voxel_cloud)
target_compile_options (aggregate_bag PRIVATE ${OVM_ROS1_COMPILE_OPTIONS})
set_target_properties (aggregate_bag PROPERTIES ${OVM_ROS1_TARGET_PROPERTIES})

# example / node executable to demonstrate runtime processing
add_executable (receding_ground_extraction_node nodes/receding_ground_extraction_node.cpp)
target_include_directories (receding_ground_extraction_node PRIVATE ${OVM_ROS1_INCLUDE_DIRS})
target_link_libraries (receding_ground_extraction_node ${OVM_ROS1_DEPENDENCIES} ros1_voxel_cloud)
target_compile_options (receding_ground_extraction_node PRIVATE ${OVM_ROS1_COMPILE_OPTIONS})
set_target_properties (receding_ground_extraction_node PROPERTIES ${OVM_ROS1_TARGET_PROPERTIES})

###############################################################################
#                                 TESTS                                       #
###############################################################################

if (OVM_BUILD_TESTS)
  add_subdirectory(tests)
endif (OVM_BUILD_TESTS)

###############################################################################
#                                INSTALL                                      #
###############################################################################

# install library build targets
install (TARGETS ros1_voxel_cloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install executable build targets
install (TARGETS aggregate_bag receding_ground_extraction_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install configuration / runtime directories
install (DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
