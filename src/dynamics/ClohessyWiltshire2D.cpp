#include "gncpy/dynamics/ClohessyWiltshire2D.h"

namespace lager::gncpy::dynamics {

Eigen::MatrixXd ClohessyWiltshire2D::getStateMat(
    [[maybe_unused]] double timestep,
    [[maybe_unused]] const StateTransParams* const stateTransParams) const {

    Eigen::MatrixXd F(4,4);
    double f11 = 4 - 3 * cos(m_mean_motion * m_dt);
    double f13 = 1 / m_mean_motion * sin(m_mean_motion * m_dt);
    double f14 = 2/m_mean_motion * (1 - cos(m_mean_motion * m_dt));
    double f21 = 6 * (sin(m_mean_motion * m_dt) - m_mean_motion * m_dt);
    double f23 = 2/m_mean_motion * (cos(m_mean_motion * m_dt) - 1);
    double f24 = 1/m_mean_motion * (4 * sin(m_mean_motion * m_dt) - 3 * m_mean_motion * m_dt);
    double f31 = 3 * m_mean_motion *sin(m_mean_motion * m_dt);
    double f33 = cos(m_mean_motion * m_dt);
    double f34 = 2 * sin(m_mean_motion * m_dt);
    double f41 = 6 * m_mean_motion * (cos(m_mean_motion * m_dt) - 1);
    double f43 = -2 * sin(m_mean_motion * m_dt);
    double f44 =  4 * cos(m_mean_motion * m_dt) - 3;


    F << f11, 0, f13, f14, f21, 1, f23, f24, f31 , 0, f33, f34, f41, 0, f43, f44;

    // Continuous Form
    //F << 3 * n^2, 0, 0, 2 * n, 0, 0, -2*n, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    return F;
    }

}