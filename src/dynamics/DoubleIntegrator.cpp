#include "gncpy/dynamics/DoubleIntegrator.h"

namespace lager::gncpy::dynamics {

Eigen::MatrixXd DoubleIntegrator::getStateMat(
    [[maybe_unused]] double timestep,
    [[maybe_unused]] const StateTransParams* const stateTransParams) const {
    Eigen::MatrixXd F(4, 4);
    F << 1, 0, m_dt, 0, 0, 1, 0, m_dt, 0, 0, 1, 0, 0, 0, 0, 1;

    return F;
}

}  // namespace lager::gncpy::dynamics
