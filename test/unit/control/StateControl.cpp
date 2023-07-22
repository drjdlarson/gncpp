#include "gncpy/control/StateControl.h"

#include <gtest/gtest.h>
#include <math.h>

#include <Eigen/Dense>

#include "gncpy/Exceptions.h"
#include "gncpy/math/Math.h"

TEST(ControlTest, StateControlGetControlInputs) {
    Eigen::Vector3d x;
    x << 3.0, 4.0, 1.0;

    Eigen::Vector3d u;
    u << 1.0, 1.0, 1.0;

    lager::gncpy::control::StateControl controller(x.size(), 3);
    std::vector<uint8_t> rows = {0, 1, 2};
    std::vector<uint8_t> cols = {0, 1, 2};

    EXPECT_THROW(controller.getControlInput(0, u, nullptr),
                 lager::gncpy::exceptions::BadParams);

    auto params = lager::gncpy::control::StateControlParams(rows, cols);

    auto out = controller.getControlInput(0, u, &params);

    EXPECT_EQ(x.size(), out.size());

    for (uint8_t ii = 0; ii < out.size(); ii++) {
        EXPECT_DOUBLE_EQ(out(ii), u(ii));
    }

    SUCCEED();
}

TEST(ControlTest, StateControlGetInputMat) {
    Eigen::Vector3d x;
    x << 3.0, 4.0, 1.0;

    lager::gncpy::control::StateControl controller(x.size(), 3);
    std::vector<uint8_t> rows = {0, 1, 2};
    std::vector<uint8_t> cols = {0, 1, 2};

    auto params = lager::gncpy::control::StateControlParams(rows, cols);

    EXPECT_THROW(controller.getInputMat(0, nullptr),
                 lager::gncpy::exceptions::BadParams);

    auto out = controller.getInputMat(0, &params);
    Eigen::Matrix3d exp({{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}});

    EXPECT_EQ(exp.rows(), out.rows());
    EXPECT_EQ(exp.cols(), out.cols());

    for (uint8_t r = 0; r < exp.rows(); r++) {
        for (uint8_t c = 0; c < exp.cols(); c++) {
            EXPECT_EQ(exp(r, c), out(r, c));
        }
    }

    SUCCEED();
}

TEST(ControlTest, StateControlGetInputMat2) {
    Eigen::VectorXd x(6);
    x << 3.0, 4.0, 1.0, 0., 0., 0.;

    lager::gncpy::control::StateControl controller(x.size(), 3);
    std::vector<uint8_t> rows = {3, 3, 4, 5};
    std::vector<uint8_t> cols = {0, 1, 1, 2};
    std::vector<double> vals = {2, 4, 6, 7};

    auto params = lager::gncpy::control::StateControlParams(rows, cols, vals);

    EXPECT_THROW(controller.getInputMat(0, nullptr),
                 lager::gncpy::exceptions::BadParams);

    auto out = controller.getInputMat(0, &params);
    Eigen::MatrixXd exp({{0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0},
                         {2.0, 4.0, 0.0},
                         {0.0, 6.0, 0.0},
                         {0.0, 0.0, 7.0}});

    EXPECT_EQ(exp.rows(), out.rows());
    EXPECT_EQ(exp.cols(), out.cols());

    for (uint8_t r = 0; r < exp.rows(); r++) {
        for (uint8_t c = 0; c < exp.cols(); c++) {
            EXPECT_EQ(exp(r, c), out(r, c));
        }
    }

    SUCCEED();
}
