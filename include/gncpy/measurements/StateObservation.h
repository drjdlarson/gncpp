#pragma once
#include <vector>

#include "gncpy/Exceptions.h"
#include "gncpy/SerializeMacros.h"
#include "gncpy/Utilities.h"
#include "gncpy/math/Math.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"
#include "gncpy/measurements/ILinearMeasModel.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

class StateObservationParams final : public MeasParams {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(StateObservationParams)

   public:
    StateObservationParams() = default;
    explicit StateObservationParams(const std::vector<uint8_t>& obsInds)
        : obsInds(obsInds) {}

    std::vector<uint8_t> obsInds;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar) {
        ar(cereal::make_nvp("MeasParams",
                            cereal::virtual_base_class<MeasParams>(this)),
           CEREAL_NVP(obsInds));
    }
};

template <typename T>
class StateObservation final : public ILinearMeasModel<T> {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(StateObservation<T>)

   public:
    StateObservation() = default;

    matrix::Matrix<T> getMeasMat(
        const matrix::Vector<T>& state,
        const MeasParams* params = nullptr) const override {
        if (!params) {
            throw exceptions::BadParams(
                "State Observation requires parameters");
        }
        if (!utilities:: instanceof <StateObservationParams>(params)) {
            throw exceptions::BadParams(
                "params type must be StateObservationParams.");
        }
        auto ptr = dynamic_cast<const StateObservationParams*>(params);
        matrix::Matrix<T> data(ptr->obsInds.size(), state.size());

        for (uint8_t ii = 0; ii < ptr->obsInds.size(); ii++) {
            for (uint8_t jj = 0; jj < state.size(); jj++) {
                if (ptr->obsInds[ii] == jj) {
                    data(ii, jj) = static_cast<T>(1.0);
                }
            }
        }
        return data;
    }

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar) {
        ar(cereal::make_nvp(
            "ILinearMeasModel",
            cereal::virtual_base_class<ILinearMeasModel<T>>(this)));
    }
};
}  // namespace lager::gncpy::measurements

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::measurements::StateObservation)
CEREAL_REGISTER_TYPE(lager::gncpy::measurements::StateObservationParams)
