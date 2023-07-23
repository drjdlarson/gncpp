#pragma once
#include <math.h>

#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <functional>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/measurements/INonLinearMeasModel.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

class RangeAndBearingParams final : public MeasParams {
    friend class boost::serialization::access;

    // GNCPY_SERIALIZE_CLASS(RangeAndBearingParams)

   public:
    RangeAndBearingParams() = default;
    RangeAndBearingParams(uint8_t xInd, uint8_t yInd)
        : xInd(xInd), yInd(yInd) {}

    uint8_t xInd;
    uint8_t yInd;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar& boost::serialization::base_object<MeasParams>(*this);
        ar& xInd;
        ar& yInd;
    }
};

class RangeAndBearing final : public INonLinearMeasModel {
    friend class boost::serialization::access;

    // GNCPY_SERIALIZE_CLASS(RangeAndBearing)

   public:
    RangeAndBearing() = default;

   protected:
    std::vector<std::function<double(const Eigen::VectorXd&)>> getMeasFuncLst(
        const MeasParams* params) const override;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar& boost::serialization::base_object<INonLinearMeasModel>(*this);
    }

    double range(const Eigen::VectorXd& state,
                 const MeasParams* params = nullptr) const;
    double bearing(const Eigen::VectorXd& state,
                   const MeasParams* params = nullptr) const;
};

}  // namespace lager::gncpy::measurements
