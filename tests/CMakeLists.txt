# Unit Test cmake configuration.

find_package(GTest REQUIRED)

# development testing executable
add_executable (development development.cpp)
target_link_libraries (development ${PROJECT_NAME})

# openvdb additions
add_subdirectory(openvdb)