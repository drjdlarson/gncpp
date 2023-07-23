#include <gtest/gtest.h>
#include <math.h>

#include <Eigen/Dense>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <sstream>

#include "gncpy/Exceptions.h"
#include "gncpy/math/Math.h"
#include "gncpy/measurements/RangeAndBearing.h"
#include "gncpy/measurements/StateObservation.h"

TEST(MeasurementTest, StateObservationParamsSerialize) {
    std::vector<uint8_t> inds{0, 1};
    lager::gncpy::measurements::StateObservationParams params(inds);

    auto classState = params.saveClassState();

    auto loadedParams =
        lager::gncpy::measurements::StateObservationParams::loadClass(
            classState);

    EXPECT_EQ(params.obsInds.size(), loadedParams.obsInds.size());

    SUCCEED();
}

TEST(MeasurementTest, StateObservationMeasure) {
    Eigen::Vector3d x;
    x << 3.0, 4.0, 1.0;
    lager::gncpy::measurements::StateObservation sensor;
    std::vector<uint8_t> inds = {0, 1, 2};

    EXPECT_THROW(sensor.measure(x, nullptr),
                 lager::gncpy::exceptions::BadParams);

    auto params = lager::gncpy::measurements::StateObservationParams(inds);

    auto out = sensor.measure(x, &params);

    EXPECT_EQ(x.size(), out.size());

    for (uint8_t ii = 0; ii < out.size(); ii++) {
        EXPECT_DOUBLE_EQ(x(ii), out(ii));
    }

    SUCCEED();
}

TEST(MeasurementTest, StateObservationMeasMat) {
    Eigen::Vector3d x;
    x << 3.0, 4.0, 1.0;
    lager::gncpy::measurements::StateObservation sensor;
    std::vector<uint8_t> inds = {0, 1, 2};
    auto params = lager::gncpy::measurements::StateObservationParams(inds);

    EXPECT_THROW(sensor.getMeasMat(x, nullptr),
                 lager::gncpy::exceptions::BadParams);

    auto out = sensor.getMeasMat(x, &params);
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

TEST(MeasurementTest, RangeBearingMeasure) {
    Eigen::Vector3d x;
    x << 3.0, 4.0, 1.0;
    lager::gncpy::measurements::RangeAndBearing sensor;
    Eigen::Vector2d exp;
    exp << 5.0, atan2(4.0, 3.0);

    EXPECT_THROW(sensor.measure(x, nullptr),
                 lager::gncpy::exceptions::BadParams);

    lager::gncpy::measurements::RangeAndBearingParams params(0, 1);

    auto out = sensor.measure(x, &params);

    // EXPECT_EQ(exp.size(), out.size());

    for (uint8_t ii = 0; ii < exp.size(); ii++) {
        EXPECT_DOUBLE_EQ(exp(ii), out(ii));
    }

    SUCCEED();
}

TEST(MeasurementTest, RangeBearingMeasMat) {
    Eigen::Vector3d x;
    x << 3.0, 4.0, 1.0;
    lager::gncpy::measurements::RangeAndBearing sensor;
    Eigen::MatrixXd exp({{0.6, 0.8, 0.0}, {-0.16, 0.12, 0.0}});

    EXPECT_THROW(sensor.getMeasMat(x), lager::gncpy::exceptions::BadParams);

    lager::gncpy::measurements::RangeAndBearingParams params(0, 1);

    auto res = sensor.getMeasMat(x, &params);

    EXPECT_EQ(exp.rows(), res.rows());
    EXPECT_EQ(exp.cols(), res.cols());

    for (uint8_t ii = 0; ii < exp.rows(); ii++) {
        for (uint8_t jj = 0; jj < exp.cols(); jj++) {
            EXPECT_NEAR(exp(ii, jj), res(ii, jj), 1e-6);
        }
    }

    SUCCEED();
}