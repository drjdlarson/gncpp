#include "gncpy/dynamics/ILinearDynamics.h"

namespace lager::gncpy::dynamics {

void ILinearDynamics::setControlModel(
    boost::shared_ptr<control::ILinearControlModel> model) {
    m_controlModel = model;
}

Eigen::VectorXd ILinearDynamics::propagateState(
    double timestep, const Eigen::VectorXd& state,
    const StateTransParams* stateTransParams) const {
    Eigen::VectorXd nextState =
        propagateState_(timestep, state, stateTransParams);

    if (hasStateConstraint()) {
        stateConstraint(timestep, nextState);
    }

    return nextState;
}

Eigen::VectorXd ILinearDynamics::propagateState(
    double timestep, const Eigen::VectorXd& state,
    const Eigen::VectorXd& control,
    const lager::gncpy::control::ControlParams* controlParams) const {
    Eigen::VectorXd nextState = propagateState_(timestep, state);

    if (hasControlModel()) {
        nextState +=
            m_controlModel->getControlInput(timestep, control, controlParams);
    } else {
        throw exceptions::BadParams(
            "Control input given but no control model set");
    }

    if (hasStateConstraint()) {
        stateConstraint(timestep, nextState);
    }

    return nextState;
}

Eigen::VectorXd ILinearDynamics::propagateState(
    double timestep, const Eigen::VectorXd& state,
    const Eigen::VectorXd& control, const StateTransParams* stateTransParams,
    const control::ControlParams* controlParams,
    const ConstraintParams* constraintParams) const {
    Eigen::VectorXd nextState =
        propagateState_(timestep, state, stateTransParams);

    if (hasControlModel()) {
        nextState +=
            m_controlModel->getControlInput(timestep, control, controlParams);
    }

    if (hasStateConstraint()) {
        stateConstraint(timestep, nextState, constraintParams);
    }

    return nextState;
}

Eigen::VectorXd ILinearDynamics::propagateState_(
    double timestep, const Eigen::VectorXd& state,
    const StateTransParams* stateTransParams) const {
    return stateTransParams == nullptr
               ? getStateMat(timestep) * state
               : getStateMat(timestep, stateTransParams) * state;
}

}  // namespace lager::gncpy::dynamics
