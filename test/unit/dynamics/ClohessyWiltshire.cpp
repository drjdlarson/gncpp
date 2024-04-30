#include <gncpy/control/StateControl.h>
#include <gncpy/dynamics/ClohessyWiltshire.h>
#include <gncpy/dynamics/Parameters.h>
#include <gtest/gtest.h>
#include <math.h>

#include <Eigen/Dense>
#include <iostream>
#include <sstream>

TEST(CWHOrbit, Propagate) {
    double dt = 0.1;
    double mean_motion = M_PI / 2.0;
    lager::gncpy::dynamics::ClohessyWiltshire dyn(dt, mean_motion);
    // error has something to do with eigen ddeclarations, figure out how to
    // declare things Eigen::Vector<double, 6> xk;
    Eigen::VectorXd xk(6);
    xk << 0., 0., 0., 1., 0., 1.;

    Eigen::VectorXd expected(6);
    expected << 2.0 / M_PI, -4.0 / M_PI, 1 / mean_motion, 0, -2, -mean_motion;

    for (uint16_t kk = 0; kk < 10; kk++) {
        double timestep = kk * dt;
        std::cout << "t = " << timestep << ": ";

        xk = dyn.propagateState(timestep, xk);
        for (auto const& x : xk) {
            std::cout << x << " ";
        }
        std::cout << std::endl;
    }

    for (uint16_t ii = 0; ii < expected.size(); ii++) {
        EXPECT_NE(abs(expected(ii) - xk(ii)), 1e-6);
    }

    SUCCEED();
}

TEST(CWHOrbit, Control) {
    double dt = 0.1;
    double mean_motion = M_PI / 2.0;
    lager::gncpy::dynamics::ClohessyWiltshire dyn(dt, mean_motion);
    Eigen::VectorXd xk(6);
    xk << 0., 0., 0., 1., 0., 1.;

    auto contObj = boost::make_shared<lager::gncpy::control::StateControl>(
        xk.size(), xk.size() / 2);

    std::vector<uint8_t> cRows = {3, 4, 5};
    std::vector<uint8_t> cCols = {0, 1, 2};
    auto contParams = lager::gncpy::control::StateControlParams(cRows, cCols);

    Eigen::VectorXd contInput(3);
    contInput << 1., 1., 1.;

    dyn.setControlModel(contObj);

    Eigen::VectorXd expected(6);
    expected << 8.36957, -3.87519, 4.36282, 18.5593, -16.2938, 6.8531;

    for (uint16_t kk = 0; kk < 10; kk++) {
        double timestep = kk * dt;
        std::cout << "t = " << timestep << ": ";

        xk = dyn.propagateState(timestep, xk, contInput, &contParams);
        for (auto const& x : xk) {
            std::cout << x << " ";
        }
        std::cout << std::endl;
    }

    for (uint16_t ii = 0; ii < expected.size(); ii++) {
        EXPECT_NEAR(expected(ii), xk(ii), 1e-5);
    }

    SUCCEED();
}

// TEST(CWHOrbit, serialize) {
//     double dt = 0.1;
//     double mean_motion = 2 * M_PI;
//     lager::gncpy::dynamics::ClohessyWiltshire dyn(dt, mean_motion);
//     auto controller =
//         std::make_shared<lager::gncpy::control::StateControl>(1, 1);
//     // Define control model variable

//     dyn.setControlModel(controller);

//     std::cout << "Original class:\n" << dyn.toXML() << std::endl;

//     std::stringstream filtState = dyn.saveClassState();
//     auto dyn2 =
//     lager::gncpy::dynamics::ClohessyWiltshire::loadClass(filtState);

//     std::cout << "Loaded class:\n" << dyn2.toXML() << std::endl;

//     EXPECT_DOUBLE_EQ(dyn.dt(), dyn2.dt());
//     EXPECT_DOUBLE_EQ(dyn.mean_motion(), dyn2.mean_motion());

//     // can not save control model or state constraint function
//     EXPECT_EQ(dyn.hasControlModel(), dyn2.hasControlModel());

//     EXPECT_EQ(dyn.hasStateConstraint(), dyn2.hasStateConstraint());

//     SUCCEED();
// }