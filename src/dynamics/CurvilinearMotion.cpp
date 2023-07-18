#include "gncpy/dynamics/CurvilinearMotion.h"

namespace lager::gncpy::dynamics {

CurvilinearMotion::CurvilinearMotion() {
    // INonLinearDynamics::setControlModel(
        // []([[maybe_unused]] double timestep,
        //    [[maybe_unused]] const Eigen::VectorXd& state,
        //    const Eigen::VectorXd& control,
        //    [[maybe_unused]] const lager::gncpy::control::ControlParams* controlParams = nullptr) {
        //     Eigen::Vector4d out;
        //     out << 0.0, 0.0, control(0), control(1);
        //     return out;
        // },
        // true);
}

Eigen::VectorXd CurvilinearMotion::continuousDynamics(
    [[maybe_unused]] double timestep, const Eigen::VectorXd& state,
    [[maybe_unused]] const StateTransParams* stateTransParams) const {
    Eigen::Vector4d out;
    out << state(2) * cos(state(3)), state(2) * sin(state(3)), 0.0, 0.0;
    return out;
}

void CurvilinearMotion::clearControlModel() {
    std::cerr << "Warning, disabling control model and it can not be "
                 "re-enabled for this curvilinear motion model!"
              << std::endl;
    INonLinearDynamics::clearControlModel();
}

}  // namespace lager::gncpy::dynamics
