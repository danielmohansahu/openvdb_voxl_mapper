###############################################################################
#                               Test Setup                                    #
###############################################################################

find_package(GTest REQUIRED)

set(OVM_ROS1_TEST_DEPENDENCIES
  GTest::GTest
  GTest::Main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ros1_voxel_cloud
)

enable_testing()

###############################################################################
#                               OVM TESTS                                     #
###############################################################################

# core test executable
add_executable(ovm_ros1_test TestOVMROS1.cpp)
target_link_libraries(ovm_ros1_test ${OVM_ROS1_TEST_DEPENDENCIES})
add_test(NAME ovm_ros1_unit_test COMMAND $<TARGET_FILE:ovm_ros1_test> -v)
