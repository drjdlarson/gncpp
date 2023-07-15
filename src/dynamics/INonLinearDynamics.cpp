#include "gncpy/dynamics/INonLinearDynamics.h"

#include "gncpy/Exceptions.h"
#include "gncpy/math/Math.h"

namespace lager::gncpy::dynamics {

void INonLinearDynamics::setControlModel
(
    std::shared_ptr<control::IControlModel> model,
    bool continuousModel
) {
    m_hasControlModel = true;
    m_continuousControl = continuousModel;
    m_controlModel = model;
}

Eigen::VectorXd INonLinearDynamics::propagateState(
    double timestep, const Eigen::VectorXd& state,
    const StateTransParams* stateTransParams) const {
    Eigen::VectorXd nextState =
        math::rungeKutta4<Eigen::VectorXd, Eigen::VectorXd, double>(
            timestep, state, dt(),
            [this, stateTransParams](double t, const Eigen::VectorXd& x) {
                return continuousDynamics(t, x, stateTransParams);
            });

    if (hasStateConstraint()) {
        stateConstraint(timestep, nextState);
    }

    return nextState;
}

Eigen::VectorXd INonLinearDynamics::propagateState(
    double timestep, const Eigen::VectorXd& state,
    const Eigen::VectorXd& control,
    const lager::gncpy::control::ControlParams* controlParams) const {
    Eigen::VectorXd nextState;
    if (m_hasControlModel && m_continuousControl) {
        nextState = math::rungeKutta4<Eigen::VectorXd, Eigen::VectorXd, double>(
            timestep, state, dt(),
            [this, &control, &controlParams](double t, const Eigen::VectorXd& x) {
                return continuousDynamics(t, x) +
                         m_controlModel->getControlInput(x, control, controlParams);
                        //  m_controlModel->getControlInput(t, x, control, controlParams);
            });
    } else if (m_hasControlModel) {
        nextState = math::rungeKutta4<Eigen::VectorXd, Eigen::VectorXd, double>(
            timestep, state, dt(), [this](double t, const Eigen::VectorXd& x) {
                return continuousDynamics(t, x);
            });

            nextState += m_controlModel->getControlInput(state, control, controlParams);
        // nextState += m_controlModel->getControlInput(timestep, state, control, controlParams);
    } else {
        throw exceptions::BadParams(
            "Control input given but no control model set");
    }

    if (hasStateConstraint()) {
        stateConstraint(timestep, nextState);
    }

    return nextState;
}

Eigen::VectorXd INonLinearDynamics::propagateState(
    double timestep, const Eigen::VectorXd& state,
    const Eigen::VectorXd& control,
    const StateTransParams* const stateTransParams,
    const lager::gncpy::control::ControlParams* const controlParams,
    const ConstraintParams* const constraintParams) const {
    Eigen::VectorXd nextState;
    if (m_hasControlModel && m_continuousControl) {
        nextState = math::rungeKutta4<Eigen::VectorXd, Eigen::VectorXd, double>(
            timestep, state, dt(),
            [this, &control, stateTransParams, controlParams](
                double t, const Eigen::VectorXd& x) {
                return continuousDynamics(t, x, stateTransParams) +
                       m_controlModel->getControlInput(x, control, controlParams);
            });
    } else if (m_hasControlModel) {
        nextState = math::rungeKutta4<Eigen::VectorXd, Eigen::VectorXd, double>(
            timestep, state, dt(), [this](double t, const Eigen::VectorXd& x) {
                return continuousDynamics(t, x);
            });

        nextState += m_controlModel->getControlInput(state, control, controlParams);
    } else {
        nextState = math::rungeKutta4<Eigen::VectorXd, Eigen::VectorXd, double>(
            timestep, state, dt(),
            [this, stateTransParams](double t, const Eigen::VectorXd& x) {
                return continuousDynamics(t, x, stateTransParams);
            });
    }

    if (hasStateConstraint()) {
        stateConstraint(timestep, nextState, constraintParams);
    }

    return nextState;
}

Eigen::MatrixXd INonLinearDynamics::getStateMat(
    double timestep, const Eigen::VectorXd& state,
    const StateTransParams* stateTransParams) const {
    return math::getJacobian(
        state,
        [this, timestep, stateTransParams](const Eigen::VectorXd& x) {
            return continuousDynamics(timestep, x, stateTransParams);
        },
        state.size());
}

}  // namespace lager::gncpy::dynamics
