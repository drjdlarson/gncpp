#include "gncpy/filters/Kalman.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "gncpy/dynamics/DoubleIntegrator.h"
#include "gncpy/dynamics/Parameters.h"
#include "gncpy/filters/Parameters.h"
#include "gncpy/measurements/Parameters.h"
#include "gncpy/measurements/StateObservation.h"

TEST(KFTest, SetStateModel) {
    double dt = 0.01;
    Eigen::MatrixXd noise(
        {{1.0, 0.0, 0, 0}, {0.0, 1.0, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});
    auto dynObj =
        std::make_shared<lager::gncpy::dynamics::DoubleIntegrator>(dt);

    lager::gncpy::filters::Kalman filt;

    filt.setStateModel(dynObj, noise);

    SUCCEED();
}

TEST(KFTest, SetMeasModel) {
    Eigen::Matrix2d noise({{1.0, 0.0}, {0.0, 1.0}});
    auto measObj =
        std::make_shared<lager::gncpy::measurements::StateObservation>();

    lager::gncpy::filters::Kalman filt;

    filt.setMeasurementModel(measObj, noise);

    SUCCEED();
}

TEST(KFTest, GetSetCovariance) {
    Eigen::Matrix4d covariance({{0.1, 0.0, 0.0, 0.0},
                                {0.0, 0.1, 0.0, 0.0},
                                {0.0, 0.0, 0.01, 0.0},
                                {0.0, 0.0, 0.0, 0.01}});
    lager::gncpy::filters::Kalman filt;

    filt.cov = covariance;
    auto newCov = filt.cov;

    for (uint8_t ii = 0; ii < covariance.rows(); ii++) {
        for (uint8_t jj = 0; jj < covariance.cols(); jj++) {
            EXPECT_EQ(newCov(ii, jj), covariance(ii, jj));
        }
    }

    SUCCEED();
}

TEST(KFTest, FilterPredict) {
    double dt = 1.0;
    Eigen::Matrix4d noise({{0.1, 0.0, 0.0, 0.0},
                           {0.0, 0.1, 0.0, 0.0},
                           {0.0, 0.0, 0.01, 0.0},
                           {0.0, 0.0, 0.0, 0.01}});

    auto dynObj =
        std::make_shared<lager::gncpy::dynamics::DoubleIntegrator>(dt);
    Eigen::Vector4d state({1.0, 2.0, 1.0, 1.0});

    Eigen::Vector2d control({0.0, 0.0});
    const Eigen::Vector4d exp({2.0, 3.0, 1.0, 1.0});

    auto predParams = lager::gncpy::filters::BayesPredictParams();
    lager::gncpy::filters::Kalman filt;
    const Eigen::Matrix4d cov({{1.0, 0.0, 0.0, 0.0},
                               {0.0, 1.0, 0.0, 0.0},
                               {0.0, 0.0, 1.0, 0.0},
                               {0.0, 0.0, 0.0, 1.0}});
    filt.cov = cov;

    filt.setStateModel(dynObj, noise);

    auto out = filt.predict(0.0, state, control, &predParams);

    for (uint8_t ii = 0; ii < exp.size(); ii++) {
        EXPECT_EQ(exp(ii), out(ii));
    }

    SUCCEED();
}

TEST(KFTest, FilterCorrect) {
    Eigen::Matrix4d noise({{0.01, 0.0, 0.0, 0.0},
                           {0.0, 0.01, 0.0, 0.0},
                           {0.0, 0.0, 0.01, 0.0},
                           {0.0, 0.0, 0.0, 0.01}});

    auto measObj =
        std::make_shared<lager::gncpy::measurements::StateObservation>();
    Eigen::Vector4d state({1.0, 2.0, 1.0, 1.0});

    const Eigen::Vector4d exp({2.0, 3.0, 1.0, 1.0});

    std::vector<uint8_t> inds = {0, 1};
    auto corrParams = lager::gncpy::filters::BayesCorrectParams();
    corrParams.measParams =
        std::make_shared<lager::gncpy::measurements::StateObservationParams>(
            inds);
    lager::gncpy::filters::Kalman filt;

    const Eigen::Matrix4d cov({{1.0, 0.0, 0.0, 0.0},
                               {0.0, 1.0, 0.0, 0.0},
                               {0.0, 0.0, 1.0, 0.0},
                               {0.0, 0.0, 0.0, 1.0}});
    filt.cov = cov;

    filt.setMeasurementModel(measObj, noise);

    auto meas = measObj->measure(exp, corrParams.measParams.get());

    double measFitProb;
    auto out = filt.correct(0.0, meas, state, measFitProb, &corrParams);

    std::cout << "corrected state estimate: \n" << out << std::endl;

    for (uint8_t ii = 0; ii < exp.size(); ii++) {
        EXPECT_NEAR(exp(ii), out(ii), 1e-6);
    }

    SUCCEED();
}

TEST(KFTest, serialize) {
    double dt = 0.2;
    const Eigen::Matrix4d cov({{1.0, 0.0, 0.3, 0.0},
                               {0.0, 1.0, 0.0, 4.0},
                               {0.0, 3.0, 1.0, 0.0},
                               {4.5, 0.0, 0.0, 1.0}});
    const Eigen::Matrix4d pNoise({{0.1, 0.0, 0.0, 0.0},
                                  {0.0, 0.1, 0.0, 0.3},
                                  {0.2, 0.0, 0.01, 0.0},
                                  {0.0, 0.0, 0.0, 0.01}});
    const Eigen::Matrix4d mNoise({{0.2, 0.0, 0.0, 0.0},
                                  {2.0, 0.4, 0.0, 0.0},
                                  {0.0, 0.0, 0.45, 2.4},
                                  {0.0, 0.0, 0.0, 0.31}});
    auto dynObj =
        std::make_shared<lager::gncpy::dynamics::DoubleIntegrator>(dt);
    auto measObj =
        std::make_shared<lager::gncpy::measurements::StateObservation>();
    lager::gncpy::filters::Kalman filt;
    filt.cov = cov;
    filt.setStateModel(dynObj, pNoise);
    filt.setMeasurementModel(measObj, mNoise);

    std::cout << "Original class:\n" << filt.toJSON() << std::endl;
    std::stringstream classState = filt.saveClassState();

    auto filt2 = lager::gncpy::filters::Kalman::loadClass(classState);
    std::cout << "Loaded class:\n" << filt2.toJSON() << std::endl;

    EXPECT_EQ(filt.cov.rows(), filt2.cov.rows());
    EXPECT_EQ(filt.cov.cols(), filt2.cov.cols());

    for (size_t r = 0; r < filt.cov.rows(); r++) {
        for (size_t c = 0; c < filt.cov.cols(); c++) {
            EXPECT_DOUBLE_EQ(filt.cov(r, c), filt2.cov(r, c));
        }
    }

    SUCCEED();
}
