# Unit Test cmake configuration.

find_package(GTest REQUIRED)

set(OVM_TEST_DEPENDENCIES
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME}
)

##### development testing executable
add_executable (development development.cpp)
target_link_libraries (development ${OVM_TEST_DEPENDENCIES})

##### openvdb additions / modifications unit tests
add_subdirectory(openvdb)

##### OpenVDB VOXL Mapper Unit Tests

set(OVM_SOURCE_FILES
  main.cc
  TestVoxelCloud.cpp
)

add_executable(ovm_test ${OVM_SOURCE_FILES})
target_link_libraries(ovm_test ${OVM_TEST_DEPENDENCIES})
add_test(NAME ovm_unit_test COMMAND $<TARGET_FILE:ovm_test> -v)
