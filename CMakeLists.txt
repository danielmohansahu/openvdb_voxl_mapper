# OpenVDB Voxel Mapping library
cmake_minimum_required (VERSION 3.5)
project (openvdb_voxel_mapping)

###############################################################################
#                 DEPENDENCIES & GLOBAL CONFIGURATION                         #
###############################################################################

# locate required dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
find_package(OpenVDB REQUIRED COMPONENTS openvdb nanovdb)

# required top level options
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED TRUE)

###############################################################################
#                                LIBRARY                                      #
###############################################################################

# build core library
add_library (${PROJECT_NAME} src/aggregator.cpp)
target_include_directories (${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenVDB_INCLUDE_DIRS})
target_link_libraries (${PROJECT_NAME} PUBLIC ${OpenVDB_LIBRARIES} OpenVDB::openvdb OpenVDB::nanovdb)
target_compile_options (${PROJECT_NAME} PRIVATE -Werror -Wall)

###############################################################################
#                                 TESTS                                       #
###############################################################################

add_executable (development tests/development.cpp)
target_link_libraries (development ${PROJECT_NAME})
