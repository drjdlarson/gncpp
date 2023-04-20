#include <gncpy/dynamics/CurvilinearMotion.h>
#include <gncpy/math/Matrix.h>
#include <gncpy/math/Vector.h>
#include <gtest/gtest.h>

#include <cmath>
#include <iostream>

TEST(Curvilinear, Propagate) {
    double dt = 0.1;
    double d2r = M_PI / 180.0;
    lager::gncpy::dynamics::CurvilinearMotion<double> dyn;
    dyn.setDt(dt);

    lager::gncpy::matrix::Vector xk({0., 0., 1., 45. * d2r});

    for (uint16_t kk = 0; kk < 10; kk++) {
        double timestep = kk * dt;
        std::cout << "t = " << timestep << ": ";

        xk = dyn.propagateState(timestep, xk);
        for (auto const& x : xk) {
            std::cout << x << " ";
        }
        std::cout << std::endl;
    }

    EXPECT_DOUBLE_EQ(xk(0), 1. * cos(45. * d2r) * 1.);
    EXPECT_DOUBLE_EQ(xk(1), 1. * sin(45. * d2r) * 1.);
    EXPECT_DOUBLE_EQ(xk(2), 1.0);
    EXPECT_DOUBLE_EQ(xk(3), 45. * d2r);

    SUCCEED();
}

TEST(Curvilinear, serialize) {
    double dt = 0.1;
    lager::gncpy::dynamics::CurvilinearMotion<double> dyn;
    dyn.setDt(dt);

    std::cout << "Original class:\n" << dyn.toJSON() << std::endl;

    std::stringstream filtState = dyn.saveClassState();
    auto dyn2 =
        lager::gncpy::dynamics::CurvilinearMotion<double>::loadClass(filtState);

    std::cout << "Loaded class:\n" << dyn2.toJSON() << std::endl;

    EXPECT_DOUBLE_EQ(dyn.dt(), dyn2.dt());

    // curvilinear motion has control hardcoded so these should still be equal
    EXPECT_EQ(dyn.hasControlModel(), dyn2.hasControlModel());

    EXPECT_EQ(dyn.hasStateConstraint(), dyn2.hasStateConstraint());

    SUCCEED();
}
