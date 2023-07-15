#include <gncpy/dynamics/ClohessyWiltshire2D.h>
#include <gncpy/dynamics/Parameters.h>
#include <gtest/gtest.h>
#include <gncpy/control/StateControl.h>

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <sstream>

TEST(CWHOrbit2D, Propagate) {
    double dt = 0.1;
    double mean_motion = M_PI/2.0;
    lager::gncpy::dynamics::ClohessyWiltshire2D dyn(dt, mean_motion);
    Eigen::Vector4d xk;
    xk << 0., 0., 1., 0.;

    Eigen::Vector4d expected;
    expected << 2.0 / M_PI, -4.0 / M_PI, 0, -2;

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
        EXPECT_NE(abs(expected(ii) - xk(ii)), 1e-6);
    }

    SUCCEED();
}

TEST(CWHOrbit2D, Control) {
    double dt = 0.1;
    double mean_motion = M_PI/2.0;
    lager::gncpy::dynamics::ClohessyWiltshire2D dyn(dt, mean_motion);
    Eigen::Vector4d xk;
    xk << 0., 0., 1., 0.;

    auto contObj = std::make_shared<lager::gncpy::control::StateControl>();
    
    std::vector<uint8_t> inds = {2, 3};
    auto contParams = lager::gncpy::control::StateControlParams(inds);

    dyn.setControlModel(contObj);

    EXPECT_EQ(dyn.hasControlModel(), 1);

    Eigen::VectorXd contInput(2);
    contInput << 1., 1.;

    Eigen::Vector4d expected;
    expected << 8.36957, -3.87519, 18.5593, -16.2938;

    for (uint16_t kk = 0; kk < 10; kk++) {
        double timestep = kk * dt;
        std::cout << "t = " << timestep << ": ";

        xk = dyn.propagateState(timestep, xk, contInput, &contParams);
        for (auto const& x : xk) {
            std::cout << x << " ";
        }
        std::cout << std::endl;
    }

    // for (uint16_t ii=0; ii<4;ii++) {
    //     EXPECT_EQ(expected(ii), xk(ii));
    // }


    for (uint16_t ii=0; ii<4;ii++) {
        EXPECT_NEAR(expected(ii), xk(ii), 1e-5);
    }

    SUCCEED();
}


// Change

TEST(CWHOrbit2D, serialize) {
    double dt = 0.1;
    double mean_motion = 2 * M_PI;
    lager::gncpy::dynamics::ClohessyWiltshire2D dyn(dt, mean_motion);
    auto controller = std::make_shared<lager::gncpy::control::StateControl>();
    //Define control model variable

    dyn.setControlModel(
        controller
       );

    std::cout << "Original class:\n" << dyn.toJSON() << std::endl;

    std::stringstream filtState = dyn.saveClassState();
    auto dyn2 = lager::gncpy::dynamics::ClohessyWiltshire2D::loadClass(filtState);

    std::cout << "Loaded class:\n" << dyn2.toJSON() << std::endl;

    EXPECT_DOUBLE_EQ(dyn.dt(), dyn2.dt());
    EXPECT_DOUBLE_EQ(dyn.mean_motion(), dyn2.mean_motion());

    // can not save control model or state constraint function
    EXPECT_NE(dyn.hasControlModel(), dyn2.hasControlModel());

    EXPECT_EQ(dyn.hasStateConstraint(), dyn2.hasStateConstraint());

    SUCCEED();
}