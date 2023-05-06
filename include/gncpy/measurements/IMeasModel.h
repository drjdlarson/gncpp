#pragma once
#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>
#include <concepts>

#include "gncpy/SerializeMacros.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

template <typename T>
    requires std::integral<T> || std::floating_point<T>
class IMeasModel {
    friend class cereal::access;

   public:
    virtual ~IMeasModel() = default;
    virtual matrix::Vector<T> measure(
        const matrix::Vector<T>& state,
        const MeasParams* params = nullptr) const = 0;
    virtual matrix::Matrix<T> getMeasMat(
        const matrix::Vector<T>& state,
        const MeasParams* params = nullptr) const = 0;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar) {
        /* nothing to save*/
    }
};

}  // namespace lager::gncpy::measurements

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::measurements::IMeasModel)
