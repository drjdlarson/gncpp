#pragma once
#include <functional>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/math/Math.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"
#include "gncpy/measurements/IMeasModel.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

template <typename T>
class INonLinearMeasModel : public IMeasModel<T> {
    friend class cereal::access;

   public:
    matrix::Vector<T> measure(
        const matrix::Vector<T>& state,
        const MeasParams* params = nullptr) const override;
    matrix::Matrix<T> getMeasMat(
        const matrix::Vector<T>& state,
        const MeasParams* params = nullptr) const override;

   protected:
    virtual std::vector<std::function<T(const matrix::Vector<T>&)>>
    getMeasFuncLst(const MeasParams* params = nullptr) const = 0;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar);
};

template <typename T>
matrix::Vector<T> INonLinearMeasModel<T>::measure(
    const matrix::Vector<T>& state, const MeasParams* params) const {
    std::vector<T> data;
    for (auto const& h : this->getMeasFuncLst(params)) {
        data.emplace_back(h(state));
    }
    return matrix::Vector<T>(data.size(), data);
}

template <typename T>
matrix::Matrix<T> INonLinearMeasModel<T>::getMeasMat(
    const matrix::Vector<T>& state, const MeasParams* params) const {
    return math::getJacobian(state, this->getMeasFuncLst(params));
}

template <typename T>
template <class Archive>
void INonLinearMeasModel<T>::serialize([[maybe_unused]] Archive& ar) {
    ar(cereal::make_nvp("IMeasModel",
                        cereal::virtual_base_class<IMeasModel<T>>(this)));
}

}  // namespace lager::gncpy::measurements

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::measurements::INonLinearMeasModel)
