if(GNCPY_COVERAGE)
  include(CodeCoverage)
endif()
#---------------------------------------------------------------------------
# setup Kalman Filter tests
#---------------------------------------------------------------------------
add_executable(
  kalman_test
  Kalman.cpp
)
target_link_libraries(
  kalman_test
  GTest::gtest_main
  Eigen3::Eigen
  lager::gncpy
)
gtest_discover_tests(kalman_test)

#---------------------------------------------------------------------------
# setup Extended Kalman Filter tests
#---------------------------------------------------------------------------
add_executable(
  extended_kalman_test
  ExtendedKalman.cpp
)
target_link_libraries(
  extended_kalman_test
  GTest::gtest_main
  Eigen3::Eigen
  lager::gncpy
)
if(GNCPY_COVERAGE)
  append_coverage_compiler_flags_to_target(extended_kalman_test)
endif()
gtest_discover_tests(extended_kalman_test)