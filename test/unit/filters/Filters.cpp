#include <gtest/gtest.h>

#include "gncpy/dynamics/DoubleIntegrator.h"
#include "gncpy/dynamics/Parameters.h"
#include "gncpy/filters/Kalman.h"
#include "gncpy/filters/Parameters.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"
#include "gncpy/measurements/Parameters.h"
#include "gncpy/measurements/StateObservation.h"

TEST(KFTest, SetStateModel) {
    double dt = 0.01;
    lager::gncpy::matrix::Matrix<double> noise(
        {{1.0, 0.0, 0, 0}, {0.0, 1.0, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});
    auto dynObj =
        std::make_shared<lager::gncpy::dynamics::DoubleIntegrator<double>>(dt);

    lager::gncpy::filters::Kalman<double> filt;

    filt.setStateModel(dynObj, noise);

    SUCCEED();
}

TEST(KFTest, SetMeasModel) {
    lager::gncpy::matrix::Matrix<double> noise({{1.0, 0.0}, {0.0, 1.0}});
    auto measObj = std::make_shared<
        lager::gncpy::measurements::StateObservation<double>>();

    lager::gncpy::filters::Kalman<double> filt;

    filt.setMeasurementModel(measObj, noise);

    SUCCEED();
}

TEST(KFTest, GetSetCovariance) {
    lager::gncpy::matrix::Matrix covariance({{0.1, 0.0, 0.0, 0.0},
                                             {0.0, 0.1, 0.0, 0.0},
                                             {0.0, 0.0, 0.01, 0.0},
                                             {0.0, 0.0, 0.0, 0.01}});
    lager::gncpy::filters::Kalman<double> filt;

    filt.cov = covariance;
    lager::gncpy::matrix::Matrix newCov = filt.cov;

    for (uint8_t ii = 0; ii < 4; ii++) {
        for (uint8_t jj = 0; jj < 4; jj++) {
            EXPECT_EQ(newCov(ii, jj), covariance(ii, jj));
        }
    }

    SUCCEED();
}

TEST(KFTest, FilterPredict) {
    double dt = 1.0;
    lager::gncpy::matrix::Matrix<double> noise({{0.1, 0.0, 0.0, 0.0},
                                                {0.0, 0.1, 0.0, 0.0},
                                                {0.0, 0.0, 0.01, 0.0},
                                                {0.0, 0.0, 0.0, 0.01}});

    auto dynObj =
        std::make_shared<lager::gncpy::dynamics::DoubleIntegrator<double>>(dt);
    lager::gncpy::matrix::Vector state({1.0, 2.0, 1.0, 1.0});

    lager::gncpy::matrix::Vector control({0.0, 0.0});
    lager::gncpy::matrix::Vector exp({2.0, 3.0, 1.0, 1.0});

    auto predParams = lager::gncpy::filters::BayesPredictParams();
    lager::gncpy::filters::Kalman<double> filt;
    lager::gncpy::matrix::Matrix cov({{1.0, 0.0, 0.0, 0.0},
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
    lager::gncpy::matrix::Matrix<double> noise({{0.1, 0.0, 0.0, 0.0},
                                                {0.0, 0.1, 0.0, 0.0},
                                                {0.0, 0.0, 0.01, 0.0},
                                                {0.0, 0.0, 0.0, 0.01}});

    auto measObj = std::make_shared<
        lager::gncpy::measurements::StateObservation<double>>();
    lager::gncpy::matrix::Vector state({1.0, 2.0, 1.0, 1.0});

    lager::gncpy::matrix::Vector exp({2.0, 3.0, 1.0, 1.0});

    std::vector<uint8_t> inds = {0, 1};
    auto corrParams = lager::gncpy::filters::BayesCorrectParams();
    corrParams.measParams =
        std::make_shared<lager::gncpy::measurements::StateObservationParams>(
            inds);
    lager::gncpy::filters::Kalman<double> filt;

    lager::gncpy::matrix::Matrix cov({{1.0, 0.0, 0.0, 0.0},
                                      {0.0, 1.0, 0.0, 0.0},
                                      {0.0, 0.0, 1.0, 0.0},
                                      {0.0, 0.0, 0.0, 1.0}});
    filt.cov = cov;

    filt.setMeasurementModel(measObj, noise);

    auto meas = measObj->measure(exp, corrParams.measParams.get());

    double measFitProb;
    auto out = filt.correct(0.0, meas, exp, measFitProb, &corrParams);

    for (uint8_t ii = 0; ii < exp.size(); ii++) {
        EXPECT_EQ(exp(ii), out(ii));
    }

    SUCCEED();
}

TEST(KFTest, serialize) {
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
        std::make_shared<lager::gncpy::dynamics::DoubleIntegrator<double>>(dt);
    auto measObj = std::make_shared<
        lager::gncpy::measurements::StateObservation<double>>();
    lager::gncpy::filters::Kalman<double> filt;
    filt.cov = cov;
    filt.setStateModel(dynObj, pNoise);
    filt.setMeasurementModel(measObj, mNoise);

    std::cout << "Original class:\n" << filt.toJSON() << std::endl;
    std::stringstream classState = filt.saveClassState();

    auto filt2 = lager::gncpy::filters::Kalman<double>::loadClass(classState);
    std::cout << "Loaded class:\n" << filt2.toJSON() << std::endl;

    EXPECT_EQ(filt.cov.numRows(), filt2.cov.numRows());
    EXPECT_EQ(filt.cov.numCols(), filt2.cov.numCols());

    for (size_t r = 0; r < filt.cov.numRows(); r++) {
        for (size_t c = 0; c < filt.cov.numCols(); c++) {
            EXPECT_DOUBLE_EQ(filt.cov(r, c), filt2.cov(r, c));
        }
    }

    SUCCEED();
}
