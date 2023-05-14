#include "gncpy/measurements/INonLinearMeasModel.h"

#include "gncpy/math/Math.h"

namespace lager::gncpy::measurements {

Eigen::VectorXd INonLinearMeasModel::measure(const Eigen::VectorXd& state,
                                             const MeasParams* params) const {
    Eigen::VectorXd out(getMeasFuncLst().size());
    size_t ind = 0;
    for (auto const& h : getMeasFuncLst(params)) {
        out(ind) = h(state);
        ind++;
    }
    return out;
}

Eigen::MatrixXd INonLinearMeasModel::getMeasMat(
    const Eigen::VectorXd& state, const MeasParams* params) const {
    return math::getJacobian(state, getMeasFuncLst(params));
}

}  // namespace lager::gncpy::measurements
