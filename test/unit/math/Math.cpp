#include "gncpy/math/Math.h"

#include <gtest/gtest.h>
#include <math.h>

#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"

TEST(MathTest, GradientVec) {
    lager::gncpy::matrix::Vector x({3., 2.7, -6.25});
    lager::gncpy::matrix::Vector u({-0.5, 2.});
    lager::gncpy::matrix::Vector exp(
        {0.42737987371310737, -21.462216395207179, 8.0999999951814061});

    auto fnc = [&u](const lager::gncpy::matrix::Vector<double>& x_) {
        return x_(0) * sin(x_(1)) + 3 * x_(2) * x_(1) + u(0);
    };
    lager::gncpy::matrix::Vector res =
        lager::gncpy::math::getGradient<double>(x, fnc);

    EXPECT_EQ(exp.size(), res.size());

    for (uint8_t ii = 0; ii < res.size(); ii++) {
        EXPECT_DOUBLE_EQ(exp(ii), res(ii));
    }

    SUCCEED();
}

TEST(MathTest, JacobianSquareMat) {
    lager::gncpy::matrix::Vector x({3., 2.7, -6.25});
    lager::gncpy::matrix::Vector u({-0.5, 2.});

    auto f0 = [&u](const lager::gncpy::matrix::Vector<double>& x_) {
        return x_(0) * sin(x_(1)) + 3. * x_(2) * x_(1) + u(0);
    };
    auto f1 = [&u](const lager::gncpy::matrix::Vector<double>& x_) {
        return x_(0) * x_(0) + 3. * x_(2) * x_(1) + u(0) * u(1);
    };
    auto f2 = [&u](const lager::gncpy::matrix::Vector<double>& x_) {
        return x_(2) * cos(x_(0)) + x_(1) * x_(1) + sin(u(0));
    };

    std::vector<
        std::function<double(const lager::gncpy::matrix::Vector<double>&)>>
        fncLst({f0, f1, f2});

    lager::gncpy::matrix::Matrix res =
        lager::gncpy::math::getJacobian<double>(x, fncLst);
    lager::gncpy::matrix::Matrix exp(
        {{0.42737987371310737, -21.462216395207179, 8.0999999951814061},
         {5.999999999062311, -18.749999988187938, 8.0999999951814061},
         {0.88200003744987043, 5.4000000027087935, -0.98999249686926305}});

    EXPECT_EQ(exp.numRows(), res.numRows());
    EXPECT_EQ(exp.numCols(), res.numCols());

    for (uint8_t r = 0; r < exp.numRows(); r++) {
        for (uint8_t c = 0; c < exp.numCols(); c++) {
            EXPECT_DOUBLE_EQ(exp(r, c), res(r, c));
        }
    }

    SUCCEED();
}

TEST(MathTest, JacobianMat) {
    lager::gncpy::matrix::Vector x({3., 2.7, -6.25});
    lager::gncpy::matrix::Vector u({-0.5, 2.});

    auto f0 = [&x](const lager::gncpy::matrix::Vector<double>& u_) {
        return x(0) * sin(x(1)) + 3. * x(2) * x(1) + u_(0);
    };
    auto f1 = [&x](const lager::gncpy::matrix::Vector<double>& u_) {
        return x(0) * x(0) + 3. * x(2) * x(1) + u_(0) * u_(1);
    };
    auto f2 = [&x](const lager::gncpy::matrix::Vector<double>& u_) {
        return x(2) * cos(x(0)) + x(1) * x(1) + sin(u_(0));
    };

    std::vector<
        std::function<double(const lager::gncpy::matrix::Vector<double>&)>>
        fncLst({f0, f1, f2});

    lager::gncpy::matrix::Matrix res =
        lager::gncpy::math::getJacobian<double>(u, fncLst);
    lager::gncpy::matrix::Matrix exp(
        {{1.0, 0.0}, {2.0, -0.5}, {0.877582562, 0.0}});

    EXPECT_EQ(exp.numRows(), res.numRows());
    EXPECT_EQ(exp.numCols(), res.numCols());

    for (uint8_t r = 0; r < exp.numRows(); r++) {
        for (uint8_t c = 0; c < exp.numCols(); c++) {
            EXPECT_NEAR(exp(r, c), res(r, c), 1e-6);
        }
    }

    SUCCEED();
}

TEST(MathTest, JacobianFunction) {
    lager::gncpy::matrix::Vector x({3.4, 2.2, 5.2});

    auto fnc = [](const lager::gncpy::matrix::Vector<double>& x_) {
        return lager::gncpy::matrix::Vector<double>(
            {x_(0) * x_(0) - x_(1) * x_(2), x_(0) * x_(1) * x_(2)});
    };

    auto jac = [](const lager::gncpy::matrix::Vector<double>& x_) {
        return lager::gncpy::matrix::Matrix(
            {{2.0 * x_(0), -x_(2), -x_(1)},
             {x_(1) * x_(2), x_(0) * x_(2), x_(0) * x_(1)}});
    };

    lager::gncpy::matrix::Matrix<double> res =
        lager::gncpy::math::getJacobian<double>(x, fnc, 2);
    lager::gncpy::matrix::Matrix exp = jac(x);

    EXPECT_EQ(exp.numRows(), res.numRows());
    EXPECT_EQ(exp.numCols(), res.numCols());

    for (uint8_t r = 0; r < exp.numRows(); r++) {
        for (uint8_t c = 0; c < exp.numCols(); c++) {
            EXPECT_NEAR(exp(r, c), res(r, c), 1e-6);
        }
    }

    SUCCEED();
}

TEST(MathTest, GaussianPDF) {
    lager::gncpy::matrix::Vector<double> x({
        0.5,
    });
    lager::gncpy::matrix::Vector<double> m({
        1,
    });
    lager::gncpy::matrix::Matrix<double> cov({{
        2,
    }});

    double res = lager::gncpy::math::calcGaussianPDF(x, m, cov);
    double exp = 0.26500353;

    EXPECT_NEAR(exp, res, 1e-8);

    SUCCEED();
}

TEST(MathTest, GaussianPDFVec) {
    lager::gncpy::matrix::Vector<double> x({0.5, 3.4});
    lager::gncpy::matrix::Vector<double> m({1, 2});
    lager::gncpy::matrix::Matrix<double> cov({{2, 0}, {0, 2}});

    double res = lager::gncpy::math::calcGaussianPDF(x, m, cov);
    double exp = 0.04579756995735449;

    EXPECT_NEAR(exp, res, 1e-8);

    SUCCEED();
}

TEST(MathTest, RungeKutta4) {
    lager::gncpy::matrix::Vector<double> x({0.5, 3.4});
    double dt = 0.1;

    auto fnc = [](double t_, const lager::gncpy::matrix::Vector<double>& x_) {
        return lager::gncpy::matrix::Vector(
            {x_(0) * x_(0), x_(0) * x_(1) * t_ - x_(1)});
    };

    auto integral = [](double t_,
                       const lager::gncpy::matrix::Vector<double>& x_) {
        return lager::gncpy::matrix::Vector(
            {-1.0 / (-2.0 + t_),
             13.6 * exp(-2.0 * t_) / ((-2.0 + t_) * (-2.0 + t_))});
    };

    lager::gncpy::matrix::Vector res =
        lager::gncpy::math::rungeKutta4<double>(0.0, x, dt, fnc);
    lager::gncpy::matrix::Vector exp = integral(dt, x);

    EXPECT_EQ(exp.size(), res.size());

    for (size_t ii = 0; ii < exp.size(); ii++) {
        EXPECT_NEAR(exp(ii), res(ii), 3e-7);
    }

    SUCCEED();
}
