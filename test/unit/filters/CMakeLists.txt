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
  lager::gncpy
)
gtest_discover_tests(extended_kalman_test)