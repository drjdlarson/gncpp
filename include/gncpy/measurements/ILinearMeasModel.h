#pragma once
#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>

#include "gncpy/SerializeMacros.h"
#include "gncpy/measurements/IMeasModel.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

class ILinearMeasModel : public IMeasModel {
    friend class boost::serialization::access;

   public:
    Eigen::VectorXd measure(const Eigen::VectorXd& state,
                            const MeasParams* params = nullptr) const override;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar) {
        ar& boost::serialization::base_object<IMeasModel>(*this);
    }
};

}  // namespace lager::gncpy::measurements
