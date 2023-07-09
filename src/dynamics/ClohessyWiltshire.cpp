#include "gncpy/dynamics/ClohessyWiltshire.h"

namespace lager::gncpy::dynamics {

Eigen::MatrixXd ClohessyWiltshire::getStateMat(
    [[maybe_unused]] double timestep,
    [[maybe_unused]] const StateTransParams* const stateTransParams) const {

    Eigen::MatrixXd F(6,6);
    double f11 = 4. - 3. * cos(m_mean_motion * m_dt);
    double f14 = 1. / m_mean_motion * sin(m_mean_motion * m_dt);
    double f15 = 2. / m_mean_motion * (1. - cos(m_mean_motion * m_dt));

    double f21 = 6. * (sin(m_mean_motion * m_dt) - m_mean_motion * m_dt);
    double f24 = 2. / m_mean_motion * (cos(m_mean_motion * m_dt) - 1.);
    double f25 = 1. / m_mean_motion * (4. * sin(m_mean_motion * m_dt) - 3. * m_mean_motion * m_dt);
    
    double f33 = cos(m_mean_motion * m_dt);
    double f36 = 1. / m_mean_motion * sin(m_mean_motion * m_dt);

    double f41 = 3. * m_mean_motion *sin(m_mean_motion * m_dt);
    double f44 = cos(m_mean_motion * m_dt);
    double f45 = 2. * sin(m_mean_motion * m_dt);
    
    double f51 = 6. * m_mean_motion * (cos(m_mean_motion * m_dt) - 1.);
    double f54 = -2. * sin(m_mean_motion * m_dt);
    double f55 =  4. * cos(m_mean_motion * m_dt) - 3.;

    double f63 = - m_mean_motion * sin(m_mean_motion * m_dt);
    double f66 = cos(m_mean_motion * m_dt);


    F << f11, 0., 0., f14, f15, 0., 
         f21, 1., 0., f24, f25, 0., 
         0., 0., f33, 0., 0., f36, 
         f41, 0., 0, f44, f45, 0., 
         f51, 0., 0., f54, f55, 0., 
         0., 0., f63, 0., 0., f66;

    return F;
}

}