#include "gncpy/control/StateControl.h"

#include "gncpy/Exceptions.h"
#include "gncpy/Utilities.h"

namespace lager::gncpy::control {

Eigen::MatrixXd StateControl::getInputMat([[maybe_unused]] double timestep,
                                          const ControlParams* params) const {

    if (!params) {
        throw exceptions::BadParams("State Control requires parameters");
    }
    if (!utilities::instanceof <StateControlParams>(params)) {
        throw exceptions::BadParams(
            "params type must be StateControlParams.");
    }
    auto ptr = dynamic_cast<const StateControlParams*>(params);
    Eigen::MatrixXd data(m_stateDim, ptr->contRows.size());

    data.fill(0.0);

    uint8_t listInd = 0;

    for (uint8_t ii=0; ii < m_stateDim; ii++){
        for (uint8_t jj=0; jj < ptr->contRows.size(); jj++)
        {
            if (ptr->contRows[listInd] == ii && ptr->contColumns[listInd]==jj) {
                data(ptr->contRows[listInd], ptr->contColumns[listInd]) = ptr->vals[listInd];
                listInd += 1;
            }
        }
    }
    return data;
}

} // namespace lager::gncpy::control