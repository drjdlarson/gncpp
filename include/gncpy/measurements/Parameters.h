#pragma once
#include <boost/serialization/access.hpp>

#include "gncpy/SerializeMacros.h"

namespace lager::gncpy::measurements {

class MeasParams {
    friend class boost::serialization::access;

    GNCPY_SERIALIZE_CLASS(MeasParams)

   public:
    virtual ~MeasParams() = default;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar, [[maybe_unused]] const unsigned int version) {
        /* nothing to serialize */
    }
};

}  // namespace lager::gncpy::measurements
