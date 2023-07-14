#---------------------------------------------------------------------------
# setup Control tests
#---------------------------------------------------------------------------
add_executable(
  control_test
  StateControl.cpp
)
target_link_libraries(
  control_test
  GTest::gtest_main
  lager::gncpy
)
gtest_discover_tests(control_test)