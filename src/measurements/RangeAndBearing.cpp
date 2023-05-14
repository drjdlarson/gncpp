#include "gncpy/measurements/RangeAndBearing.h"

#include "gncpy/Exceptions.h"
#include "gncpy/Utilities.h"

namespace lager::gncpy::measurements {

std::vector<std::function<double(const Eigen::VectorXd&)>>
RangeAndBearing::getMeasFuncLst(const MeasParams* params) const {
    auto h1 = [this, params](const Eigen::VectorXd& x) {
        return this->range(x, params);
    };
    auto h2 = [this, params](const Eigen::VectorXd& x) {
        return this->bearing(x, params);
    };
    return std::vector<std::function<double(const Eigen::VectorXd&)>>({h1, h2});
}

double RangeAndBearing::range(const Eigen::VectorXd& state,
                              const MeasParams* params) const {
    if (!params) {
        throw exceptions::BadParams("Range and Bearing requires parameters.");
    }
    if (!utilities:: instanceof <RangeAndBearingParams>(params)) {
        throw exceptions::BadParams(
            "params type must be RangeAndBearingParams.");
    }
    auto ptr = dynamic_cast<const RangeAndBearingParams*>(params);

    return sqrt(state(ptr->xInd) * state(ptr->xInd) +
                state(ptr->yInd) * state(ptr->yInd));
}

double RangeAndBearing::bearing(const Eigen::VectorXd& state,
                                const MeasParams* params) const {
    if (!params) {
        throw exceptions::BadParams("Range and Bearing requires parameters.");
    }
    if (!utilities:: instanceof <RangeAndBearingParams>(params)) {
        throw exceptions::BadParams(
            "params type must be RangeAndBearingParams.");
    }
    auto ptr = dynamic_cast<const RangeAndBearingParams*>(params);

    return atan2(state(ptr->yInd), state(ptr->xInd));
}

}  // namespace lager::gncpy::measurements
