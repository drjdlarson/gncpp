#pragma once
#include <Eigen/Dense>

#include "gncpy/SerializeMacros.h"
#include "gncpy/control/IControlModel.h"
#include "gncpy/control/Parameters.h"

namespace lager::gncpy::control {

class ILinearControlModel : public IControlModel {
    friend class cereal::access;

    public:
      Eigen::VectorXd getControlInput(double timestep,
                                      const Eigen::VectorXd& input,
                                      const ControlParams* params=nullptr) const;
      virtual Eigen::MatrixXd getInputMat(double timestep,
            const ControlParams* params = nullptr) const = 0;
    private:
      template <class Archive>
      void serialize([[maybe_unused]] Archive& ar);

};

template <class Archive>
void ILinearControlModel::serialize([[maybe_unused]] Archive& ar) {
    ar(cereal::make_nvp("IControlModel",
                        cereal::virtual_base_class<IControlModel>(this)));
}
}  //  namespace lager:;gncpy::control

CEREAL_REGISTER_TYPE(lager::gncpy::control::ILinearControlModel)