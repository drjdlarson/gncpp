#include "gncpy/filters/ExtendedKalman.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <math.h>

#include "gncpy/dynamics/CurvilinearMotion.h"

#include "gncpy/measurements/RangeAndBearing.h"
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

TEST(EKFTest, SetMeasModel) {
    Eigen::Matrix2d noise({{1.0, 0.0}, {0.0, 1.0 * M_PI/180}});
    auto measObj =
        std::make_shared<lager::gncpy::measurements::RangeAndBearing>();

    lager::gncpy::filters::ExtendedKalman filt;

    filt.setMeasurementModel(measObj, noise);

    SUCCEED();
}

TEST(EKFTest, FilterPredict) {
    double dt = 1.0;
    Eigen::Matrix4d noise({{0.1, 0.0, 0.0, 0.0},
                           {0.0, 0.1, 0.0, 0.0},
                           {0.0, 0.0, 0.01, 0.0},
                           {0.0, 0.0, 0.0, 0.01}});

    lager::gncpy::dynamics::CurvilinearMotion dyn;

    auto dynObj =
        std::make_shared<lager::gncpy::dynamics::CurvilinearMotion>(dt);
    Eigen::Vector4d state({1.0, 2.0, 1.0, M_PI / 2});

    Eigen::Vector2d control({0.0, 0.0});
    
    const Eigen::Vector4d exp({1.0, 3.0, 1.0, M_PI / 2});//({1.0, 2.0, 1.0, M_PI / 2});

    auto predParams = lager::gncpy::filters::BayesPredictParams();
    lager::gncpy::filters::ExtendedKalman filt;
    const Eigen::Matrix4d cov({{1.0, 0.0, 0.0, 0.0},
                               {0.0, 1.0, 0.0, 0.0},
                               {0.0, 0.0, 1.0, 0.0},
                               {0.0, 0.0, 0.0, 1.0 * M_PI / 180}});
    filt.getCov() = cov;

    filt.setStateModel(dynObj, noise);

    auto out = filt.predict(0.0, state, control, &predParams);

    for (uint8_t ii = 0; ii < exp.size(); ii++) {
        EXPECT_EQ(exp(ii), out(ii));        
    }

    SUCCEED();
}

TEST(EKFTest, FilterCorrect) {
    Eigen::Matrix4d noise({{0.01, 0.0, 0.0, 0.0},
                           {0.0, 0.01, 0.0, 0.0},
                           {0.0, 0.0, 0.01, 0.0},
                           {0.0, 0.0, 0.0, 0.01}});

    auto measObj =
        std::make_shared<lager::gncpy::measurements::RangeAndBearing>();
    // Eigen::Vector4d state({1.0, 2.0, 1.0, 1.0});
    Eigen::Vector4d state({2.0, 3.0, 1.0, 1.0});

    const Eigen::Vector4d exp({2.0, 3.0, 1.0, 1.0});

    auto corrParams = lager::gncpy::filters::BayesCorrectParams();
    corrParams.measParams =
        std::make_shared<lager::gncpy::measurements::RangeAndBearingParams>(
            0, 1);
    lager::gncpy::filters::ExtendedKalman filt;

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


// TEST(EKFTest, serialize) {
//     double dt = 0.2;
//     Eigen::Matrix4d cov({{1.0, 0.0, 0.3, 0.0},
//                          {0.0, 1.0, 0.0, 4.0},
//                          {0.0, 3.0, 1.0, 0.0},
//                          {4.5, 0.0, 0.0, 1.0}});
//     Eigen::Matrix4d pNoise({{0.1, 0.0, 0.0, 0.0},
//                             {0.0, 0.1, 0.0, 0.3},
//                             {0.2, 0.0, 0.01, 0.0},
//                             {0.0, 0.0, 0.0, 0.01}});
//     Eigen::Matrix4d mNoise({{0.2, 0.0, 0.0, 0.0},
//                             {2.0, 0.4, 0.0, 0.0},
//                             {0.0, 0.0, 0.45, 2.4},
//                             {0.0, 0.0, 0.0, 0.31}});
//     auto dynObj = std::make_shared<lager::gncpy::dynamics::CurvilinearMotion>();
//     dynObj->setDt(dt);
//     lager::gncpy::filters::ExtendedKalman filt;
//     filt.getCov() = cov;
//     filt.setStateModel(dynObj, pNoise);

//     std::cout << "Original class:\n" << filt.toJSON() << std::endl;
//     std::stringstream classState = filt.saveClassState();

//     auto filt2 = lager::gncpy::filters::ExtendedKalman::loadClass(classState);
//     std::cout << "Loaded class:\n" << filt2.toJSON() << std::endl;

//     EXPECT_EQ(filt.viewCov().rows(), filt2.viewCov().rows());
//     EXPECT_EQ(filt.viewCov().cols(), filt2.viewCov().cols());

//     for (size_t r = 0; r < filt.viewCov().rows(); r++) {
//         for (size_t c = 0; c < filt.viewCov().cols(); c++) {
//             EXPECT_DOUBLE_EQ(filt.viewCov()(r, c), filt2.viewCov()(r, c));
//         }
//     }

//     EXPECT_DOUBLE_EQ(
//         std::dynamic_pointer_cast<lager::gncpy::dynamics::CurvilinearMotion>(
//             filt.dynamicsModel())
//             ->dt(),
//         std::dynamic_pointer_cast<lager::gncpy::dynamics::CurvilinearMotion>(
//             filt2.dynamicsModel())
//             ->dt());

//     SUCCEED();
// }
