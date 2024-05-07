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
        boost::make_shared<lager::gncpy::dynamics::DoubleIntegrator>(dt);

    lager::gncpy::filters::Kalman filt;

    filt.setStateModel(dynObj, noise);

    SUCCEED();
}

TEST(KFTest, SetMeasModel) {
    Eigen::Matrix2d noise({{1.0, 0.0}, {0.0, 1.0}});
    auto measObj =
        boost::make_shared<lager::gncpy::measurements::StateObservation>();

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

    filt.getCov() = covariance;
    auto newCov = filt.viewCov();

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
        boost::make_shared<lager::gncpy::dynamics::DoubleIntegrator>(dt);
    Eigen::Vector4d state({1.0, 2.0, 1.0, 1.0});

    Eigen::Vector2d control({0.0, 0.0});
    const Eigen::Vector4d exp({2.0, 3.0, 1.0, 1.0});

    auto predParams = lager::gncpy::filters::BayesPredictParams();
    lager::gncpy::filters::Kalman filt;
    const Eigen::Matrix4d cov({{1.0, 0.0, 0.0, 0.0},
                               {0.0, 1.0, 0.0, 0.0},
                               {0.0, 0.0, 1.0, 0.0},
                               {0.0, 0.0, 0.0, 1.0}});
    filt.getCov() = cov;

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
        boost::make_shared<lager::gncpy::measurements::StateObservation>();
    Eigen::Vector4d state({1.0, 2.0, 1.0, 1.0});

    const Eigen::Vector4d exp({2.0, 3.0, 1.0, 1.0});

    std::vector<uint8_t> inds = {0, 1};
    auto corrParams = lager::gncpy::filters::BayesCorrectParams();
    corrParams.measParams =
        boost::make_shared<lager::gncpy::measurements::StateObservationParams>(
            inds);
    lager::gncpy::filters::Kalman filt;

    const Eigen::Matrix4d cov({{1.0, 0.0, 0.0, 0.0},
                               {0.0, 1.0, 0.0, 0.0},
                               {0.0, 0.0, 1.0, 0.0},
                               {0.0, 0.0, 0.0, 1.0}});
    filt.getCov() = cov;

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

// TEST(KFTest, serialize) {
//     double dt = 0.2;
//     const Eigen::Matrix4d cov({{1.0, 0.0, 0.3, 0.0},
//                                {0.0, 1.0, 0.0, 4.0},
//                                {0.0, 3.0, 1.0, 0.0},
//                                {4.5, 0.0, 0.0, 1.0}});
//     const Eigen::Matrix4d pNoise({{0.1, 0.0, 0.0, 0.0},
//                                   {0.0, 0.1, 0.0, 0.3},
//                                   {0.2, 0.0, 0.01, 0.0},
//                                   {0.0, 0.0, 0.0, 0.01}});
//     const Eigen::Matrix4d mNoise({{0.2, 0.0, 0.0, 0.0},
//                                   {2.0, 0.4, 0.0, 0.0},
//                                   {0.0, 0.0, 0.45, 2.4},
//                                   {0.0, 0.0, 0.0, 0.31}});
//     auto dynObj =
//         boost::make_shared<lager::gncpy::dynamics::DoubleIntegrator>(dt);
//     std::vector<uint8_t> obsIndices = {0, 1};
//     auto measObj =
//         boost::make_shared<lager::gncpy::measurements::StateObservation>();
//     lager::gncpy::filters::Kalman filt;
//     filt.getCov() = cov;
//     filt.setStateModel(dynObj, pNoise);
//     filt.setMeasurementModel(measObj, mNoise);

//     // std::cout << "Original class:\n" << filt.toXML() << std::endl;
//     std::stringstream classState = filt.saveClassState();

//     auto filt2 = lager::gncpy::filters::Kalman::loadClass(classState);
//     // std::cout << "Loaded class:\n" << filt2.toXML() << std::endl;

//     EXPECT_EQ(filt.viewCov().rows(), filt2.viewCov().rows());
//     EXPECT_EQ(filt.viewCov().cols(), filt2.viewCov().cols());

//     for (size_t r = 0; r < filt.viewCov().rows(); r++) {
//         for (size_t c = 0; c < filt.viewCov().cols(); c++) {
//             EXPECT_DOUBLE_EQ(filt.viewCov()(r, c), filt2.viewCov()(r, c));
//         }
//     }

//     // std::cout << boost::dynamic_pointer_cast<
//     //                  lager::gncpy::dynamics::DoubleIntegrator>(
//     //                  filt.dynamicsModel())
//     //                  ->dt();
//     // std::cout << boost::dynamic_pointer_cast<
//     //                  lager::gncpy::dynamics::DoubleIntegrator>(
//     //                  filt2.dynamicsModel())
//     //                  ->dt();

//     // EXPECT_DOUBLE_EQ(
//     // boost::dynamic_pointer_cast<lager::gncpy::dynamics::DoubleIntegrator>(
//     //         filt.dynamicsModel())
//     //         ->dt(),
//     // boost::dynamic_pointer_cast<lager::gncpy::dynamics::DoubleIntegrator>(
//     //         filt2.dynamicsModel())
//     //         ->dt());

//     // EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<
//     //                      lager::gncpy::measurements::StateObservation>(
//     //                      filt.measurementModel()),
//     //                  boost::dynamic_pointer_cast<
//     //                      lager::gncpy::measurements::StateObservation>(
//     //                      filt2.measurementModel()));

//     SUCCEED();
// }
