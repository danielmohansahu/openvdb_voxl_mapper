###############################################################################
#                               Test Setup                                    #
###############################################################################

find_package(GTest REQUIRED)

set(OVM_TEST_DEPENDENCIES
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME}
)

###############################################################################
#                               OVM TESTS                                     #
###############################################################################

# development testing executable - subject to change, not a real test.
add_executable (development development.cpp)
target_link_libraries (development ${OVM_TEST_DEPENDENCIES})

set(OVM_SOURCE_FILES
  main.cc
  TestOVMConversions.cpp
  TestOVMCore.cpp
  TestOVMOperations.cpp
)

# core test executable
add_executable(ovm_test ${OVM_SOURCE_FILES})
target_link_libraries(ovm_test ${OVM_TEST_DEPENDENCIES})
add_test(NAME ovm_unit_test COMMAND $<TARGET_FILE:ovm_test> -v)

###############################################################################
#                          OpenVDB Extension TESTS                            #
###############################################################################

add_subdirectory(openvdb)
