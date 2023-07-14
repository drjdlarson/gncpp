#include "gncpy/control/ILinearControlModel.h"

namespace lager::gncpy::control {
Eigen::VectorXd ILinearControlModel::getControlInput(const Eigen::VectorXd& state,
                                                     const Eigen::VectorXd& input,
                                                     const ControlParams* params) const {
    return getInputMat(state, params) * input;
}
}  //  namespace lager::gncpy::measurements