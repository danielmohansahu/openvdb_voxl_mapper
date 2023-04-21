# OpenVDB Voxel Mapping library
cmake_minimum_required (VERSION 3.24)
project (openvdb_voxel_mapping LANGUAGES CXX CUDA)

###############################################################################
#                                  OPTIONS                                    #
###############################################################################

include(CMakeDependentOption)
option (OVM_BUILD_ROS1 "Enable ROS1 bindings." ON)
option (OVM_BUILD_TESTS "Build and run unit tests." ON)
option (OVM_BUILD_BENCHMARK "Build benchmark executables." ON)

###############################################################################
#                 DEPENDENCIES & GLOBAL CONFIGURATION                         #
###############################################################################

# locate required dependencies
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
find_package (OpenVDB 10.0.1 REQUIRED COMPONENTS openvdb nanovdb)
find_package (Eigen3 3.3.7 REQUIRED NO_MODULE)
find_package (PCL 1.10.0 REQUIRED COMPONENTS common)

if (OVM_BUILD_ROS1)

  # specify ROS package dependencies
  set (ROS1_PACKAGE_BUILD_DEPS
    grid_map_core grid_map_msgs grid_map_ros
    pcl_conversions pcl_ros
    rosbag
    sensor_msgs
    tf2_ros
  )
  set (ROS1_PACKAGE_DEPS
    ${ROS1_PACKAGE_BUILD_DEPS}
    rviz
    grid_map_rviz_plugin
  )

  # find catkin and required dependencies
  find_package (catkin REQUIRED COMPONENTS ${ROS1_PACKAGE_BUILD_DEPS})

  # configure ROS1 package metadata
  catkin_package (
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${ROS1_PACKAGE_DEPS}
    DEPENDS EIGEN PCL OpenVDB
  )
endif (OVM_BUILD_ROS1)

# deliberately load CUDAToolkit _after_ ROS; there's some CMake conflicts
find_package (CUDAToolkit REQUIRED)

# required top level options
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED TRUE)
set (CMAKE_CUDA_STANDARD 17)
set (CMAKE_CUDA_STANDARD_REQUIRED TRUE)

# default architecture target
if (NOT CMAKE_CUDA_ARCHITECTURES)
  set (CMAKE_CUDA_ARCHITECTURES native)
endif ()

# default architecture target
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE RelWithDebInfo)
  set (CMAKE_CXX_FLAGS_RelWithDebInfo "-ggdb")
endif ()

###############################################################################
#                                 DEBUG                                       #
###############################################################################

# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

###############################################################################
#                              OVM LIBRARY                                    #
###############################################################################

set (OVM_SOURCE_FILES
  src/voxel_cloud.cpp
  src/operations/ground_plane_extraction.cpp
  src/operations/ground_plane_extraction.cu
)

set (OVM_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenVDB_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
  # cmake hack - this is intended to satisfy an Eigen plugin required by grid_map
  #              note that we load it at the end of the list for low priorities
  ${catkin_INCLUDE_DIRS}
)

# build core library
add_library (${PROJECT_NAME} ${OVM_SOURCE_FILES})
target_include_directories (${PROJECT_NAME} PUBLIC ${OVM_INCLUDE_DIRS})
target_link_libraries (
  ${PROJECT_NAME}
  PUBLIC
  ${PCL_LIBARIES}
  OpenVDB::openvdb
  OpenVDB::nanovdb
  Eigen3::Eigen
  CUDA::cudart
)
target_compile_options (
  ${PROJECT_NAME}
  PUBLIC
  "$<$<COMPILE_LANGUAGE:CUDA>:SHELL:--compiler-options -Wall>"
  "$<$<COMPILE_LANGUAGE:CXX>:-Werror;-Wall;-fconcepts>"
)

###############################################################################
#                         ROS1 Configuration                                  #
###############################################################################

if (OVM_BUILD_ROS1)
  # include actual build targets, defined in subdirectory
  add_subdirectory(ros1)
endif (OVM_BUILD_ROS1)

###############################################################################
#                                 TESTS                                       #
###############################################################################

if (OVM_BUILD_TESTS)
  add_subdirectory(tests)
endif (OVM_BUILD_TESTS)

###############################################################################
#                               BENCHMARK                                     #
###############################################################################

if (OVM_BUILD_BENCHMARK)
  add_subdirectory(benchmark)
endif (OVM_BUILD_BENCHMARK)

###############################################################################
#                                INSTALL                                      #
###############################################################################

# install core library
install (TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# install headers
install (DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
