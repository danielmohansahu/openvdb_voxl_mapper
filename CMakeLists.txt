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
find_package (CUDAToolkit REQUIRED)

# required top level options
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED TRUE)
set (CMAKE_CUDA_STANDARD 17)
set (CMAKE_CUDA_STANDARD_REQUIRED TRUE)

# default architecture target
# @TODO generalize, or upgrade CMAKE to use (all, all-major, native)
if (NOT CMAKE_CUDA_ARCHITECTURES)
  set (CMAKE_CUDA_ARCHITECTURES 75)
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
#                                LIBRARY                                      #
###############################################################################

set (SOURCE_FILES
  src/openvdb_voxel_mapper.cpp
  src/operations/ground_plane_extraction.cpp
  src/operations/ground_plane_extraction.cu
)

# build core library
add_library (${PROJECT_NAME} ${SOURCE_FILES})
target_include_directories (
  ${PROJECT_NAME}
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenVDB_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)
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
#                                 TESTS                                       #
###############################################################################

add_subdirectory(tests)
