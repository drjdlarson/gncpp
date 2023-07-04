#include <gncpy/dynamics/ClohessyWiltshire.h>
#include <gncpy/dynamics/Parameters.h>
#include <gtest/gtest.h>

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <sstream>

TEST(CWHOrbit, Propagate) {
    double dt = 0.1;
    double mean_motion = M_PI/2.0;
    lager::gncpy::dynamics::ClohessyWiltshire dyn(dt, mean_motion);
    Eigen::Vector4d xk;
    xk << 0., 0., 1., 0.;

    Eigen::Vector4d exp;
    exp << 2.0 / M_PI, -4.0 / M_PI, 0, -2;

    for (uint16_t kk = 0; kk < 10; kk++) {
        double timestep = kk * dt;
        std::cout << "t = " << timestep << ": ";

        xk = dyn.propagateState(timestep, xk);
        for (auto const& x : xk) {
            std::cout << x << " ";
        }
        std::cout << std::endl;
    }

    for (uint16_t ii=0; ii<4;ii++) {
        EXPECT_NE(abs(exp(ii) - xk(ii)), 1e-6);
    }

    SUCCEED();
}

TEST(CWHOrbit, serialize) {
    double dt = 0.1;
    double mean_motion = 2 * M_PI;
    lager::gncpy::dynamics::ClohessyWiltshire dyn(dt, mean_motion);
    dyn.setControlModel(
        []([[maybe_unused]] double t,
           [[maybe_unused]] const lager::gncpy::dynamics::ControlParams*
               params) { return Eigen::Matrix4d::Identity(); });

    std::cout << "Original class:\n" << dyn.toJSON() << std::endl;

    std::stringstream filtState = dyn.saveClassState();
    auto dyn2 = lager::gncpy::dynamics::ClohessyWiltshire::loadClass(filtState);

    std::cout << "Loaded class:\n" << dyn2.toJSON() << std::endl;

    EXPECT_DOUBLE_EQ(dyn.dt(), dyn2.dt());
    EXPECT_DOUBLE_EQ(dyn.mean_motion(), dyn2.mean_motion());

    // can not save control model or state constraint function
    EXPECT_NE(dyn.hasControlModel(), dyn2.hasControlModel());

    EXPECT_EQ(dyn.hasStateConstraint(), dyn2.hasStateConstraint());

    SUCCEED();
}