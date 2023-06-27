#include "gncpy/dynamics/ClohessyWiltshire.h"

namespace lager::gncpy::dynamics {

Eigen::MatrixXd ClohessyWiltshire::getStateMat(
    [[maybe_unused]] double timestep,
    [[maybe_unused]] const StateTransParams* const stateTransParams) const {

    Eigen::MatrixXd F(4,4);
    n = 0;

    F << (4 - 3 * cos(m_mean_motion * m_dt), 0, 1 / m_mean_motion * sin(m_mean_motion * m_dt), 2/m_mean_motion * (1 - cos(m_mean_motion * m_dt)),
          6 * (sin(m_mean_motion * m_dt) - m_mean_motion * m_dt), 1, 2/m_mean_motion * (cos(m_mean_motion * m_dt) - 1), 1/m_mean_motion * (4 * sin(m_mean_motion * m_dt) - 3 * m_mean_motion * dt),
          3 * m_mean_motion *sin(m_mean_motion * m_dt), 0, cos(m_mean_motion * m_dt), 2 * sin(m_mean_motion * m_dt), 
          6 * m_mean_motion * (cos(m_mean_motion * m_dt) - 1), 0, -2 * sin(m_mean_motion * m_dt), 4 * cos(m_mean_motion * m_dt) - 3);

    // Continuous Form
    //F << 3 * n^2, 0, 0, 2 * n, 0, 0, -2*n, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    return F;
    }

}