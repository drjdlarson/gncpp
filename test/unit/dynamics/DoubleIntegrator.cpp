#include <gncpy/dynamics/DoubleIntegrator.h>
#include <gncpy/dynamics/Parameters.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <iostream>
#include <sstream>

TEST(DoubleInt, Propagate) {
    double dt = 0.1;
    lager::gncpy::dynamics::DoubleIntegrator dyn(dt);
    Eigen::Vector4d xk;
    xk << 0., 0., 1., 0.;

    for (uint16_t kk = 0; kk < 10; kk++) {
        double timestep = kk * dt;
        std::cout << "t = " << timestep << ": ";

        xk = dyn.propagateState(timestep, xk);
        for (auto const& x : xk) {
            std::cout << x << " ";
        }
        std::cout << std::endl;
    }

    EXPECT_DOUBLE_EQ(xk(0), 1.);

    SUCCEED();
}

TEST(DoubleInt, serialize) {
    double dt = 0.1;
    lager::gncpy::dynamics::DoubleIntegrator dyn(dt);
    dyn.setControlModel(
        []([[maybe_unused]] double t,
           [[maybe_unused]] const lager::gncpy::dynamics::ControlParams*
               params) { return Eigen::Matrix4d::Identity(); });

    std::cout << "Original class:\n" << dyn.toJSON() << std::endl;

    std::stringstream filtState = dyn.saveClassState();
    auto dyn2 = lager::gncpy::dynamics::DoubleIntegrator::loadClass(filtState);

    std::cout << "Loaded class:\n" << dyn2.toJSON() << std::endl;

    EXPECT_DOUBLE_EQ(dyn.dt(), dyn2.dt());

    // can not save control model or state constraint function
    EXPECT_NE(dyn.hasControlModel(), dyn2.hasControlModel());

    EXPECT_EQ(dyn.hasStateConstraint(), dyn2.hasStateConstraint());

    SUCCEED();
}