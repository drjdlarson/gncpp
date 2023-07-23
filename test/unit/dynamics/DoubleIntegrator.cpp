#include <gncpy/control/StateControl.h>
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

TEST(DoubleInt, Control) {
    double dt = 0.1;
    lager::gncpy::dynamics::DoubleIntegrator dyn(dt);
    Eigen::Vector4d xk;
    xk << 0., 0., 1., 0.;

    auto contObj = std::make_shared<lager::gncpy::control::StateControl>(
        xk.size(), xk.size() / 2);

    std::vector<uint8_t> cRows = {2, 3};
    std::vector<uint8_t> cCols = {0, 1};
    auto contParams = lager::gncpy::control::StateControlParams(cRows, cCols);

    dyn.setControlModel(contObj);

    EXPECT_EQ(dyn.hasControlModel(), 1);

    Eigen::VectorXd contInput(2);
    contInput << 1., 1.;

    for (uint16_t kk = 0; kk < 10; kk++) {
        double timestep = kk * dt;
        std::cout << "t = " << timestep << ": ";

        xk = dyn.propagateState(timestep, xk, contInput, &contParams);
        for (auto const& x : xk) {
            std::cout << x << " ";
        }
        std::cout << std::endl;
    }

    Eigen::Vector4d exp;
    exp << 5.5, 4.5, 11, 10;

    for (uint8_t ii = 0; ii < 4; ii++) {
        EXPECT_DOUBLE_EQ(exp(ii), xk(ii));
    }

    SUCCEED();
}

TEST(DoubleInt, serialize) {
    double dt = 0.1;
    lager::gncpy::dynamics::DoubleIntegrator dyn(dt);
    auto controller =
        std::make_shared<lager::gncpy::control::StateControl>(1, 1);
    // Define control model variable

    dyn.setControlModel(controller);

    // std::cout << "Original class:\n" << dyn.toXML() << std::endl;

    std::stringstream filtState = dyn.saveClassState();
    // std::cout << "Filt State Is Good Baby";
    auto dyn2 = lager::gncpy::dynamics::DoubleIntegrator::loadClass(filtState);

    // std::cout << "Loaded class:\n" << dyn2.toXML() << std::endl;

    EXPECT_DOUBLE_EQ(dyn.dt(), dyn2.dt());

    // can not save control model or state constraint function
    // EXPECT_EQ(dyn.hasControlModel(), dyn2.hasControlModel());

    // EXPECT_EQ(dyn.hasStateConstraint(), dyn2.hasStateConstraint());

    SUCCEED();
}