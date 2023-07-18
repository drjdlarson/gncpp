#include "gncpy/control/ILinearControlModel.h"

namespace lager::gncpy::control {
Eigen::VectorXd ILinearControlModel::getControlInput(double timestep,
                                                     const Eigen::VectorXd& input,
                                                     const ControlParams* params) const {
    return getInputMat(timestep, params) * input;
}
}  //  namespace lager::gncpy::measurements