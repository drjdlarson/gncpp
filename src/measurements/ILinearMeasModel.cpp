#include "gncpy/measurements/ILinearMeasModel.h"

namespace lager::gncpy::measurements {

Eigen::VectorXd ILinearMeasModel::measure(const Eigen::VectorXd& state,
                                          const MeasParams* params) const {
    return getMeasMat(state, params) * state;
}

}  // namespace lager::gncpy::measurements
