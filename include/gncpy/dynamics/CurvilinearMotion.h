#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// #include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/INonLinearDynamics.h"
#include "gncpy/dynamics/Parameters.h"

#include "gncpy/control/Parameters.h"
#include "gncpy/control/IControlModel.h"

namespace lager::gncpy::dynamics {

/**
 * @brief Implements a Curvilinear motion model
 *
 * This is a slight variation from normal INonLinearDynamics classes
 * because it hardcodes a control model. Also, the angle state should be kept
 * between 0-360 degrees. This implements the following system of ODEs
 *
 * \f{align}{
 *      \dot{x} &= v cos(\psi) \\
 *      \dot{y} &= v sin(\psi) \\
 *      \dot{v} &= u_0 \\
 *      \dot{\psi} &= u_1
 * \f}
 *
 * See \cite Li2000_SurveyofManeuveringTargetTrackingDynamicModels for details.
 *
 */
class CurvilinearMotion final : public INonLinearDynamics {
    // friend class cereal::access;

    // GNCPY_SERIALIZE_CLASS(CurvilinearMotion)

   public:
    CurvilinearMotion();
    explicit CurvilinearMotion(double dt) {setDt(dt);};

    std::vector<std::string> stateNames() const override {
        return std::vector<std::string>{"x pos", "y pos", "speed",
                                        "turn angle"};
    }

    Eigen::VectorXd continuousDynamics(
        [[maybe_unused]] double timestep, const Eigen::VectorXd& state,
        [[maybe_unused]] const StateTransParams* stateTransParams =
            nullptr) const override;
    // template <typename F>
    // void setControlModel(std::shared_ptr<lager::gncpy::control::IControlModel> model,
    //                      bool continuousModel) override ;
    void clearControlModel() override;

   private:
    // template <class Archive>
    // void serialize(Archive& ar);
    double m_dt;
    // std::shared_ptr<lager::gncpy::control::IControlModel> m_controlModel;
};

// template <typename F>
// void CurvilinearMotion::setControlModel(
//     std::shared_ptr<lager::gncpy::control::IControlModel> model,
//     bool continuousModel
//     ) {
//     std::cerr << "Can not set control model for curvilinear motion "
//                  "dynamics. It has a fixed control model!"
//               << std::endl;
// }

// template <class Archive>
// void CurvilinearMotion::serialize(Archive& ar) {
//     ar(cereal::make_nvp("INonLinearDynamics",
//                         cereal::virtual_base_class<INonLinearDynamics>(this)));
// }

}  // namespace lager::gncpy::dynamics

// CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::CurvilinearMotion)
