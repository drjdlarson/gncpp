#pragma once
#include <Eigen/Dense>
#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>

#include "gncpy/control/Parameters.h"

namespace lager::gncpy::control {
    
class IControlModel {
    friend class cereal::access;

    public:
        virtual ~IControlModel() = default;
        virtual Eigen::VectorXd getControlInput(
            const Eigen::VectorXd& state,
            const Eigen::VectorXd& input,
            const ControlParams* params = nullptr) const = 0;
        virtual Eigen::MatrixXd getInputMat(
            const Eigen::VectorXd& state,
            const ControlParams* params = nullptr) const = 0;

    private:
        template <class Archive>
        void serialize(Archive& ar);
};

template <class Archive>
void IControlModel::serialize([[maybe_unused]] Archive& ar) {
    /* nothing to save*/
}

}  //  namespace lager::gncpy::control

CEREAL_REGISTER_TYPE(lager::gncpy::control::IControlModel)