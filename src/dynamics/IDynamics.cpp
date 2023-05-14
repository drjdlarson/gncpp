#include "gncpy/dynamics/IDynamics.h"

namespace lager::gncpy::dynamics {

void IDynamics::stateConstraint(
    double timestep, Eigen::VectorXd& state,
    const ConstraintParams* const constraintParams) const {
    if (m_hasStateConstraint) {
        m_stateConstraints(timestep, state, constraintParams);
    }
    throw NoStateConstraintError();
}

}  // namespace lager::gncpy::dynamics
