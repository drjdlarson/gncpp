#include "gncpy/filters/ExtendedKalman.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "gncpy/dynamics/CurvilinearMotion.h"
#include "gncpy/dynamics/Parameters.h"
#include "gncpy/filters/Parameters.h"
#include "gncpy/measurements/Parameters.h"

TEST(EKFTest, SetStateModel) {
    Eigen::Matrix4d noise(
        {{1.0, 0.0, 0, 0}, {0.0, 1.0, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});
    auto dynObj = std::make_shared<lager::gncpy::dynamics::CurvilinearMotion>();

    lager::gncpy::filters::ExtendedKalman filt;

    filt.setStateModel(dynObj, noise);

    SUCCEED();
}

TEST(EKFTest, serialize) {
    double dt = 0.2;
    Eigen::Matrix4d cov({{1.0, 0.0, 0.3, 0.0},
                         {0.0, 1.0, 0.0, 4.0},
                         {0.0, 3.0, 1.0, 0.0},
                         {4.5, 0.0, 0.0, 1.0}});
    Eigen::Matrix4d pNoise({{0.1, 0.0, 0.0, 0.0},
                            {0.0, 0.1, 0.0, 0.3},
                            {0.2, 0.0, 0.01, 0.0},
                            {0.0, 0.0, 0.0, 0.01}});
    Eigen::Matrix4d mNoise({{0.2, 0.0, 0.0, 0.0},
                            {2.0, 0.4, 0.0, 0.0},
                            {0.0, 0.0, 0.45, 2.4},
                            {0.0, 0.0, 0.0, 0.31}});
    auto dynObj = std::make_shared<lager::gncpy::dynamics::CurvilinearMotion>();
    dynObj->setDt(dt);
    lager::gncpy::filters::ExtendedKalman filt;
    filt.cov = cov;
    filt.setStateModel(dynObj, pNoise);

    std::cout << "Original class:\n" << filt.toJSON() << std::endl;
    std::stringstream classState = filt.saveClassState();

    auto filt2 = lager::gncpy::filters::ExtendedKalman::loadClass(classState);
    std::cout << "Loaded class:\n" << filt2.toJSON() << std::endl;

    EXPECT_EQ(filt.cov.rows(), filt2.cov.rows());
    EXPECT_EQ(filt.cov.cols(), filt2.cov.cols());

    for (size_t r = 0; r < filt.cov.rows(); r++) {
        for (size_t c = 0; c < filt.cov.cols(); c++) {
            EXPECT_DOUBLE_EQ(filt.cov(r, c), filt2.cov(r, c));
        }
    }

    EXPECT_DOUBLE_EQ(
        std::dynamic_pointer_cast<lager::gncpy::dynamics::CurvilinearMotion>(
            filt.dynamicsModel())
            ->dt(),
        std::dynamic_pointer_cast<lager::gncpy::dynamics::CurvilinearMotion>(
            filt2.dynamicsModel())
            ->dt());

    SUCCEED();
}