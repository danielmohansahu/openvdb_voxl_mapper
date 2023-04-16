# OpenVDB Voxel Mapping library
cmake_minimum_required (VERSION 3.5)
project (openvdb_voxel_mapping LANGUAGES CXX CUDA)

###############################################################################
#                 DEPENDENCIES & GLOBAL CONFIGURATION                         #
###############################################################################

# locate required dependencies
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
find_package (OpenVDB REQUIRED COMPONENTS openvdb nanovdb)
find_package (Eigen3 REQUIRED NO_MODULE)
find_package (PCL REQUIRED COMPONENTS common)

# required top level options
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED TRUE)

###############################################################################
#                                LIBRARY                                      #
###############################################################################

# build core library
add_library (${PROJECT_NAME} src/openvdb_voxel_mapper.cpp)
target_include_directories (
  ${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenVDB_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries (
  ${PROJECT_NAME} PUBLIC
  ${OpenVDB_LIBRARIES}
  ${PCL_LIBARIES}
  OpenVDB::openvdb
  OpenVDB::nanovdb
  Eigen3::Eigen
)
target_compile_options (${PROJECT_NAME} PUBLIC -Werror -Wall -fconcepts)

###############################################################################
#                                 TESTS                                       #
###############################################################################

add_subdirectory(tests)
