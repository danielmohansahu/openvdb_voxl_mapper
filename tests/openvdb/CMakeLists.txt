# Copyright Contributors to the OpenVDB Project
# SPDX-License-Identifier: MPL-2.0
#
#[=======================================================================[

  CMake Configuration for OpenVDB Unit Tests
  
#]=======================================================================]

message(STATUS "----------------------------------------------------")
message(STATUS "----------- Configuring OpenVDBUnitTests -----------")
message(STATUS "----------------------------------------------------")

find_package(GTest REQUIRED)

set(OPENVDB_TEST_DEPENDENT_LIBS
  OpenVDB::openvdb
  GTest::GTest
  GTest::Main
  Blosc::blosc
  ZLIB::ZLIB
  ${PROJECT_NAME}
)

##########################################################################

##### VDB unit tests

set(UNITTEST_SOURCE_FILES
  main.cc
  TestPointMerge.cc
)

add_executable(vdb_test ${UNITTEST_SOURCE_FILES})
target_link_libraries(vdb_test ${OPENVDB_TEST_DEPENDENT_LIBS})
add_test(NAME vdb_unit_test COMMAND $<TARGET_FILE:vdb_test> -v)

set_tests_properties(vdb_unit_test PROPERTIES
    ENVIRONMENT
      "$<$<CONFIG:UBSAN>:UBSAN_OPTIONS=halt_on_error=1 report_error_type=1 suppressions=${UBSAN_SUPRESSION_FILE}>")

