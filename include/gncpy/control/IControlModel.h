#pragma once
#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>

#include "gncpy/control/Parameters.h"

namespace lager::gncpy::control {

class IControlModel {
    friend class boost::serialization::access;

   public:
    virtual ~IControlModel() = default;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar,
                   [[maybe_unused]] const unsigned int version) {
        /* nothing to serialize*/
    }
};

// BOOST_SERIALIZATION_ASSUME_ABSTRACT(IControlModel)

}  //  namespace lager::gncpy::control
