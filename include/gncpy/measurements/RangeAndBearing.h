#pragma once
#include <math.h>

#include <Eigen/Dense>
#include <functional>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/measurements/INonLinearMeasModel.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

class RangeAndBearingParams final : public MeasParams {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(RangeAndBearingParams)

   public:
    RangeAndBearingParams() = default;
    RangeAndBearingParams(uint8_t xInd, uint8_t yInd)
        : xInd(xInd), yInd(yInd) {}

    uint8_t xInd;
    uint8_t yInd;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("MeasParams",
                            cereal::virtual_base_class<MeasParams>(this)),
           CEREAL_NVP(xInd), CEREAL_NVP(yInd));
    }
};

class RangeAndBearing final : public INonLinearMeasModel {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(RangeAndBearing)

   public:
    RangeAndBearing() = default;

   protected:
    std::vector<std::function<double(const Eigen::VectorXd&)>> getMeasFuncLst(
        const MeasParams* params) const override;

   private:
    template <class Archive>
    void serialize(Archive& ar);

    double range(const Eigen::VectorXd& state,
                 const MeasParams* params = nullptr) const;
    double bearing(const Eigen::VectorXd& state,
                   const MeasParams* params = nullptr) const;
};

template <class Archive>
void RangeAndBearing::serialize(Archive& ar) {
    ar(cereal::make_nvp("INonLinearMeasModel",
                        cereal::virtual_base_class<INonLinearMeasModel>(this)));
}

}  // namespace lager::gncpy::measurements

CEREAL_REGISTER_TYPE(lager::gncpy::measurements::RangeAndBearing)
CEREAL_REGISTER_TYPE(lager::gncpy::measurements::RangeAndBearingParams)
