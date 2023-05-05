#pragma once
#include "gncpy/SerializeMacros.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"
#include "gncpy/measurements/IMeasModel.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {
template <typename T>
class ILinearMeasModel : public IMeasModel<T> {
    friend class cereal::access;

   public:
    matrix::Vector<T> measure(
        const matrix::Vector<T>& state,
        const MeasParams* params = nullptr) const override {
        return this->getMeasMat(state, params) * state;
    }

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar) {
        ar(cereal::make_nvp("IMeasModel",
                            cereal::virtual_base_class<IMeasModel<T>>(this)));
    }
};

}  // namespace lager::gncpy::measurements

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::measurements::ILinearMeasModel)