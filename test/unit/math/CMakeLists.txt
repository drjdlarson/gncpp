#---------------------------------------------------------------------------
# setup Math tests
#---------------------------------------------------------------------------
add_executable(
  math_test
  Math.cpp
)
target_link_libraries(
  math_test
  GTest::gtest_main
  Eigen3::Eigen
  lager::gncpy
)
gtest_discover_tests(math_test)