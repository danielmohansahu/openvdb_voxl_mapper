###############################################################################
#                            Benchmark Setup                                  #
###############################################################################

# sanity check that something exists for us to work with
if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/data/v1.0-mini/")
  message (FATAL_ERROR "No data found in ${CMAKE_CURRENT_SOURCE_DIR}/data/); \
                        please rerun with -DOVM_BUILD_BENCHMARK=OFF")
endif ()

set(OVM_BENCHMARK_DEPENDENCIES
  ${PROJECT_NAME}
)

###############################################################################
#                               BENCHMARK                                     #
###############################################################################

# development testing executable - subject to change, not a real test.
add_executable (nuscenes_benchmark nuscenes_benchmark.cpp)
target_link_libraries (nuscenes_benchmark ${OVM_BENCHMARK_DEPENDENCIES})
