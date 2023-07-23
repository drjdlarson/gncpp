#include "gncpy/control/ILinearControlModel.h"

#include <boost/serialization/export.hpp>

namespace lager::gncpy::control {
Eigen::VectorXd ILinearControlModel::getControlInput(
    double timestep, const Eigen::VectorXd& input,
    const ControlParams* params) const {
    return getInputMat(timestep, params) * input;
}

}  // namespace lager::gncpy::control
