#include "gncpy/measurements/StateObservation.h"

#include "gncpy/Exceptions.h"
#include "gncpy/Utilities.h"

namespace lager::gncpy::measurements {

Eigen::MatrixXd StateObservation::getMeasMat(const Eigen::VectorXd& state,
                                             const MeasParams* params) const {
    if (!params) {
        throw exceptions::BadParams("State Observation requires parameters");
    }
    if (!utilities:: instanceof <StateObservationParams>(params)) {
        throw exceptions::BadParams(
            "params type must be StateObservationParams.");
    }
    auto ptr = dynamic_cast<const StateObservationParams*>(params);
    Eigen::MatrixXd data(ptr->obsInds.size(), state.size());

    for (uint8_t ii = 0; ii < ptr->obsInds.size(); ii++) {
        for (uint8_t jj = 0; jj < state.size(); jj++) {
            if (ptr->obsInds[ii] == jj) {
                data(ii, jj) = 1.0;
            } else {
                data(ii, jj) = 0.0;
            }
        }
    }
    return data;
}

}  // namespace lager::gncpy::measurements
