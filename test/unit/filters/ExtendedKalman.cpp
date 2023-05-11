#include "gncpy/filters/ExtendedKalman.h"

#include <gtest/gtest.h>

#include "gncpy/dynamics/CurvilinearMotion.h"
#include "gncpy/dynamics/Parameters.h"
#include "gncpy/filters/Parameters.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"
#include "gncpy/measurements/Parameters.h"

TEST(EKFTest, SetStateModel) {
    lager::gncpy::matrix::Matrix<double> noise(
        {{1.0, 0.0, 0, 0}, {0.0, 1.0, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});
    auto dynObj =
        std::make_shared<lager::gncpy::dynamics::CurvilinearMotion<double>>();

    lager::gncpy::filters::ExtendedKalman<double> filt;

    filt.setStateModel(dynObj, noise);

    SUCCEED();
}

TEST(EKFTest, serialize) {
    double dt = 0.2;
    lager::gncpy::matrix::Matrix cov({{1.0, 0.0, 0.3, 0.0},
                                      {0.0, 1.0, 0.0, 4.0},
                                      {0.0, 3.0, 1.0, 0.0},
                                      {4.5, 0.0, 0.0, 1.0}});
    lager::gncpy::matrix::Matrix<double> pNoise({{0.1, 0.0, 0.0, 0.0},
                                                 {0.0, 0.1, 0.0, 0.3},
                                                 {0.2, 0.0, 0.01, 0.0},
                                                 {0.0, 0.0, 0.0, 0.01}});
    lager::gncpy::matrix::Matrix<double> mNoise({{0.2, 0.0, 0.0, 0.0},
                                                 {2.0, 0.4, 0.0, 0.0},
                                                 {0.0, 0.0, 0.45, 2.4},
                                                 {0.0, 0.0, 0.0, 0.31}});
    auto dynObj =
        std::make_shared<lager::gncpy::dynamics::CurvilinearMotion<double>>();
    dynObj->setDt(dt);
    lager::gncpy::filters::ExtendedKalman<double> filt;
    filt.cov = cov;
    filt.setStateModel(dynObj, pNoise);

    std::cout << "Original class:\n" << filt.toJSON() << std::endl;
    std::stringstream classState = filt.saveClassState();

    auto filt2 =
        lager::gncpy::filters::ExtendedKalman<double>::loadClass(classState);
    std::cout << "Loaded class:\n" << filt2.toJSON() << std::endl;

    EXPECT_EQ(filt.cov.numRows(), filt2.cov.numRows());
    EXPECT_EQ(filt.cov.numCols(), filt2.cov.numCols());

    for (size_t r = 0; r < filt.cov.numRows(); r++) {
        for (size_t c = 0; c < filt.cov.numCols(); c++) {
            EXPECT_DOUBLE_EQ(filt.cov(r, c), filt2.cov(r, c));
        }
    }

    EXPECT_DOUBLE_EQ(std::dynamic_pointer_cast<
                         lager::gncpy::dynamics::CurvilinearMotion<double>>(
                         filt.dynamicsModel())
                         ->dt(),
                     std::dynamic_pointer_cast<
                         lager::gncpy::dynamics::CurvilinearMotion<double>>(
                         filt2.dynamicsModel())
                         ->dt());

    SUCCEED();
}