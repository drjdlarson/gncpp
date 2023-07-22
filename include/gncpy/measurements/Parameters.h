#pragma once
// #include <cereal/access.hpp>
// #include <cereal/archives/binary.hpp>
// #include <cereal/archives/json.hpp>
// #include <cereal/archives/portable_binary.hpp>
// #include <cereal/types/polymorphic.hpp>

// #include "gncpy/SerializeMacros.h"

namespace lager::gncpy::measurements {

class MeasParams {
    // friend class cereal::access;

    // GNCPY_SERIALIZE_CLASS(MeasParams)

   public:
    virtual ~MeasParams() = default;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar) {
        /* nothing to serialize */
    }
};

}  // namespace lager::gncpy::measurements

// CEREAL_REGISTER_TYPE(lager::gncpy::measurements::MeasParams)
