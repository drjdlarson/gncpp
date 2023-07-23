#pragma once
#include <boost/serialization/access.hpp>
#include "gncpy/SerializeMacros.h"

namespace lager::gncpy::control {

class ControlParams {
    friend class boost::serialization::access;

    // GNCPY_SERIALIZE_CLASS(ControlParams)

   public:
    virtual ~ControlParams() = default;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar) {
        /* nothing to serialize*/
    }
};

}  //  namespace lager::gncpy::control
