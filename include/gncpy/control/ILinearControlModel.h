#pragma once
#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>

#include "gncpy/SerializeMacros.h"
#include "gncpy/control/IControlModel.h"
#include "gncpy/control/Parameters.h"

namespace lager::gncpy::control {

class ILinearControlModel : public IControlModel {
    friend class boost::serialization::access;

   public:
    Eigen::VectorXd getControlInput(
        double timestep, const Eigen::VectorXd& input,
        const ControlParams* params = nullptr) const;
    virtual Eigen::MatrixXd getInputMat(
        double timestep, const ControlParams* params = nullptr) const = 0;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar) {
        ar& boost::serialization::base_object<IControlModel>(*this);
    }
};

}  // namespace lager::gncpy::control
