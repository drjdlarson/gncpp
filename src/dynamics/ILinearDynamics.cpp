#include "gncpy/dynamics/ILinearDynamics.h"

namespace lager::gncpy::dynamics {

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
    const Eigen::VectorXd& control) const {
    Eigen::VectorXd nextState = propagateState_(timestep, state);

    if (hasControlModel()) {
        nextState += getInputMat(timestep) * control;
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
    const ControlParams* controlParams,
    const ConstraintParams* constraintParams) const {
    Eigen::VectorXd nextState =
        propagateState_(timestep, state, stateTransParams);

    if (hasControlModel()) {
        nextState += getInputMat(timestep, controlParams) * control;
    }

    if (hasStateConstraint()) {
        stateConstraint(timestep, nextState, constraintParams);
    }

    return nextState;
}

Eigen::MatrixXd ILinearDynamics::getInputMat(
    double timestep, const ControlParams* controlParams) const {
    return controlParams == nullptr ? controlModel(timestep)
                                    : controlModel(timestep, controlParams);
}

Eigen::MatrixXd ILinearDynamics::controlModel(
    double timestep, const ControlParams* controlParams) const {
    if (m_hasContolModel) {
        return m_controlModel(timestep, controlParams);
    }
    throw NoControlError();
}

Eigen::VectorXd ILinearDynamics::propagateState_(
    double timestep, const Eigen::VectorXd& state,
    const StateTransParams* stateTransParams) const {
    return stateTransParams == nullptr
               ? getStateMat(timestep) * state
               : getStateMat(timestep, stateTransParams) * state;
}

}  // namespace lager::gncpy::dynamics
