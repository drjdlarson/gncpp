#pragma once
#include <Eigen/Dense>
#include <boost/serialization/access.hpp>

#include "gncpy/control/Parameters.h"

namespace lager::gncpy::control {

class IControlModel {
    friend class boost::serialization::access;

   public:
    virtual ~IControlModel() = default;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar) {
        /* nothing to serialize*/
    }
};

}  //  namespace lager::gncpy::control
