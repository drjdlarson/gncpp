#pragma once
#include <math.h>

#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>
#include <functional>
#include <vector>

#include "gncpy/Exceptions.h"
#include "gncpy/SerializeMacros.h"
#include "gncpy/Utilities.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"
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

template <typename T>
class RangeAndBearing final : public INonLinearMeasModel<T> {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(RangeAndBearing<T>)

   public:
    RangeAndBearing() = default;

   protected:
    std::vector<std::function<T(const matrix::Vector<T>&)>> getMeasFuncLst(
        const MeasParams* params) const override;

   private:
    template <class Archive>
    void serialize(Archive& ar);

    static T range(const matrix::Vector<T>& state,
                   const MeasParams* params = nullptr);
    static T bearing(const matrix::Vector<T>& state,
                     const MeasParams* params = nullptr);
};

template <typename T>
std::vector<std::function<T(const matrix::Vector<T>&)>>
RangeAndBearing<T>::getMeasFuncLst(const MeasParams* params) const {
    auto h1 = [this, params](const matrix::Vector<T>& x) {
        return this->range(x, params);
    };
    auto h2 = [this, params](const matrix::Vector<T>& x) {
        return this->bearing(x, params);
    };
    return std::vector<std::function<T(const matrix::Vector<T>&)>>({h1, h2});
}

template <typename T>
template <class Archive>
void RangeAndBearing<T>::serialize(Archive& ar) {
    ar(cereal::make_nvp(
        "INonLinearMeasModel",
        cereal::virtual_base_class<INonLinearMeasModel<T>>(this)));
}

template <typename T>
T RangeAndBearing<T>::range(const matrix::Vector<T>& state,
                            const MeasParams* params) {
    if (!params) {
        throw exceptions::BadParams("Range and Bearing requires parameters.");
    }
    if (!utilities:: instanceof <RangeAndBearingParams>(params)) {
        throw exceptions::BadParams(
            "params type must be RangeAndBearingParams.");
    }
    auto ptr = dynamic_cast<const RangeAndBearingParams*>(params);

    return sqrt(state(ptr->xInd) * state(ptr->xInd) +
                state(ptr->yInd) * state(ptr->yInd));
}

template <typename T>
T RangeAndBearing<T>::bearing(const matrix::Vector<T>& state,
                              const MeasParams* params) {
    if (!params) {
        throw exceptions::BadParams("Range and Bearing requires parameters.");
    }
    if (!utilities:: instanceof <RangeAndBearingParams>(params)) {
        throw exceptions::BadParams(
            "params type must be RangeAndBearingParams.");
    }
    auto ptr = dynamic_cast<const RangeAndBearingParams*>(params);

    return atan2(state(ptr->yInd), state(ptr->xInd));
}

extern template class RangeAndBearing<float>;
extern template class RangeAndBearing<double>;

}  // namespace lager::gncpy::measurements

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::measurements::RangeAndBearing)
CEREAL_REGISTER_TYPE(lager::gncpy::measurements::RangeAndBearingParams)
