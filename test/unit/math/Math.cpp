#include "gncpy/math/Math.h"

#include <gtest/gtest.h>
#include <math.h>

#include <Eigen/Dense>

TEST(MathTest, GradientVec) {
    Eigen::Vector3d x;
    x << 3., 2.7, -6.25;
    Eigen::Vector2d u;
    u << -0.5, 2.;
    Eigen::Vector3d exp;
    exp << 0.42737987371310737, -21.462216395207179, 8.0999999951814061;

    auto fnc = [&u](const Eigen::Vector3d& x_) {
        return x_(0) * sin(x_(1)) + 3 * x_(2) * x_(1) + u(0);
    };
    auto res = lager::gncpy::math::getGradient(x, fnc);

    EXPECT_EQ(exp.size(), res.size());

    for (uint8_t ii = 0; ii < res.size(); ii++) {
        EXPECT_DOUBLE_EQ(exp(ii), res(ii));
    }

    SUCCEED();
}

TEST(MathTest, JacobianSquareMat) {
    Eigen::Vector3d x;
    x << 3., 2.7, -6.25;
    Eigen::Vector2d u;
    u << -0.5, 2.;

    auto f0 = [&u](const Eigen::Vector3d& x_) {
        return x_(0) * sin(x_(1)) + 3. * x_(2) * x_(1) + u(0);
    };
    auto f1 = [&u](const Eigen::Vector3d& x_) {
        return x_(0) * x_(0) + 3. * x_(2) * x_(1) + u(0) * u(1);
    };
    auto f2 = [&u](const Eigen::Vector3d& x_) {
        return x_(2) * cos(x_(0)) + x_(1) * x_(1) + sin(u(0));
    };

    std::vector<std::function<double(const Eigen::VectorXd&)>> fncLst(
        {f0, f1, f2});

    auto res = lager::gncpy::math::getJacobian(x, fncLst);
    Eigen::Matrix3d exp;
    exp << 0.42737987371310737, -21.462216395207179, 8.0999999951814061,
        5.999999999062311, -18.749999988187938, 8.0999999951814061,
        0.88200003744987043, 5.4000000027087935, -0.98999249686926305;

    EXPECT_EQ(exp.rows(), res.rows());
    EXPECT_EQ(exp.cols(), res.cols());

    for (uint8_t r = 0; r < exp.rows(); r++) {
        for (uint8_t c = 0; c < exp.cols(); c++) {
            EXPECT_DOUBLE_EQ(exp(r, c), res(r, c));
        }
    }

    SUCCEED();
}

TEST(MathTest, JacobianMat) {
    Eigen::Vector3d x;
    x << 3., 2.7, -6.25;
    Eigen::Vector2d u;
    u << -0.5, 2.;

    auto f0 = [&x](const Eigen::Vector2d& u_) {
        return x(0) * sin(x(1)) + 3. * x(2) * x(1) + u_(0);
    };
    auto f1 = [&x](const Eigen::Vector2d& u_) {
        return x(0) * x(0) + 3. * x(2) * x(1) + u_(0) * u_(1);
    };
    auto f2 = [&x](const Eigen::Vector2d& u_) {
        return x(2) * cos(x(0)) + x(1) * x(1) + sin(u_(0));
    };

    std::vector<std::function<double(const Eigen::VectorXd&)>> fncLst(
        {f0, f1, f2});

    auto res = lager::gncpy::math::getJacobian(u, fncLst);
    Eigen::MatrixXd exp(3, 2);
    exp << 1.0, 0.0, 2.0, -0.5, 0.877582562, 0.0;

    EXPECT_EQ(exp.rows(), res.rows());
    EXPECT_EQ(exp.cols(), res.cols());

    for (uint8_t r = 0; r < exp.rows(); r++) {
        for (uint8_t c = 0; c < exp.cols(); c++) {
            EXPECT_NEAR(exp(r, c), res(r, c), 1e-6);
        }
    }

    SUCCEED();
}

TEST(MathTest, JacobianFunction) {
    Eigen::Vector3d x({3.4, 2.2, 5.2});

    auto fnc = [](const Eigen::Vector3d& x_) {
        Eigen::Vector2d out;
        out << x_(0) * x_(0) - x_(1) * x_(2), x_(0) * x_(1) * x_(2);
        return out;
    };

    auto jac = [](const Eigen::Vector3d& x_) {
        Eigen::MatrixXd out(2, 3);
        out << 2.0 * x_(0), -x_(2), -x_(1), x_(1) * x_(2), x_(0) * x_(2),
            x_(0) * x_(1);
        return out;
    };

    auto res = lager::gncpy::math::getJacobian(x, fnc, 2);
    auto exp = jac(x);

    EXPECT_EQ(exp.rows(), res.rows());
    EXPECT_EQ(exp.cols(), res.cols());

    for (uint8_t r = 0; r < exp.rows(); r++) {
        for (uint8_t c = 0; c < exp.cols(); c++) {
            EXPECT_NEAR(exp(r, c), res(r, c), 1e-6);
        }
    }

    SUCCEED();
}

TEST(MathTest, GaussianPDF) {
    Eigen::VectorXd x({{0.5}});
    Eigen::VectorXd m({{1.0}});
    Eigen::MatrixXd cov({{2.0}});

    double res = lager::gncpy::math::calcGaussianPDF(x, m, cov);
    double exp = 0.26500353;

    EXPECT_NEAR(exp, res, 1e-8);

    SUCCEED();
}

TEST(MathTest, GaussianPDFVec) {
    Eigen::Vector2d x;
    x << 0.5, 3.4;
    Eigen::Vector2d m;
    m << 1, 2;
    Eigen::Matrix2d cov;
    cov << 2, 0, 0, 2;

    double res = lager::gncpy::math::calcGaussianPDF(x, m, cov);
    double exp = 0.04579756995735449;

    EXPECT_NEAR(exp, res, 1e-8);

    SUCCEED();
}

TEST(MathTest, RungeKutta4) {
    Eigen::Vector2d x;
    x << 0.5, 3.4;
    double dt = 0.1;

    auto fnc = [](double t_, const Eigen::Vector2d& x_) {
        Eigen::Vector2d out;
        out << x_(0) * x_(0), x_(0) * x_(1) * t_ - x_(1);
        return out;
    };

    auto integral = [](double t_, const Eigen::Vector2d& x_) {
        Eigen::Vector2d out;
        out << -1.0 / (-2.0 + t_),
            13.6 * exp(-2.0 * t_) / ((-2.0 + t_) * (-2.0 + t_));
        return out;
    };

    auto res = lager::gncpy::math::rungeKutta4<Eigen::Vector2d, Eigen::Vector2d,
                                               double>(0.0, x, dt, fnc);
    auto exp = integral(dt, x);

    EXPECT_EQ(exp.size(), res.size());

    for (size_t ii = 0; ii < exp.size(); ii++) {
        EXPECT_NEAR(exp(ii), res(ii), 3e-7);
    }

    SUCCEED();
}
