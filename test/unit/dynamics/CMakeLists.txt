#---------------------------------------------------------------------------
# setup Double Integrator dynamics tests
#---------------------------------------------------------------------------
add_executable(
  double_int_dyn_test
  DoubleIntegrator.cpp
)
target_link_libraries(
  double_int_dyn_test
  GTest::gtest_main
  Eigen3::Eigen
  lager::gncpy
)
gtest_discover_tests(double_int_dyn_test)

#---------------------------------------------------------------------------
# setup Curvilinear Motion dynamics tests
#---------------------------------------------------------------------------
add_executable(
  curvilinear_dyn_test
  CurvilinearMotion.cpp
)
target_link_libraries(
  curvilinear_dyn_test
  GTest::gtest_main
  Eigen3::Eigen
  lager::gncpy
)
gtest_discover_tests(curvilinear_dyn_test)

#---------------------------------------------------------------------------
# setup Clohessy Wiltshire dynamics tests
#---------------------------------------------------------------------------

add_executable(
  cwhorbit2d_dyn_test
  ClohessyWiltshire2D.cpp
)
target_link_libraries(
  cwhorbit2d_dyn_test
  GTest::gtest_main
  Eigen3::Eigen
  lager::gncpy
)
gtest_discover_tests(cwhorbit2d_dyn_test)

add_executable(
  cwhorbit_dyn_test
  ClohessyWiltshire.cpp
)
target_link_libraries(
  cwhorbit_dyn_test
  GTest::gtest_main
  Eigen3::Eigen
  lager::gncpy
)
gtest_discover_tests(cwhorbit_dyn_test)