#include "gncpy/filters/ExtendedKalman.h"

#include "gncpy/Utilities.h"
#include "gncpy/dynamics/ILinearDynamics.h"
#include "gncpy/dynamics/INonLinearDynamics.h"
#include "gncpy/math/Math.h"

namespace lager::gncpy::filters {

Eigen::VectorXd ExtendedKalman::predict(
    double timestep, const Eigen::VectorXd& curState,
    [[maybe_unused]] const std::optional<Eigen::VectorXd> controlInput,
    const BayesPredictParams* params) {
    if (params != nullptr && !utilities:: instanceof
        <BayesPredictParams>(params)) {
        throw exceptions::BadParams("Params must be BayesPredictParams");
    }

    Eigen::VectorXd nextState = dynamicsModel()->propagateState(
        timestep, curState, params->stateTransParams.get());

    Eigen::MatrixXd stateMat;
    if (utilities:: instanceof
        <dynamics::INonLinearDynamics>(dynamicsModel())) {
        stateMat = std::dynamic_pointer_cast<dynamics::INonLinearDynamics>(
                       dynamicsModel())
                       ->getStateMat(timestep, curState,
                                     params->stateTransParams.get());
    } else if (utilities:: instanceof
               <dynamics::ILinearDynamics>(dynamicsModel())) {
        stateMat = std::dynamic_pointer_cast<dynamics::ILinearDynamics>(
                       dynamicsModel())
                       ->getStateMat(timestep, params->stateTransParams.get());
    } else {
        throw exceptions::TypeError("Unknown dynamics type");
    }

    if (m_continuousCov) {
        throw std::runtime_error(
            "Continuous covariance model not implemented yet for the EKF");
    } else {
        getCov() = stateMat * viewCov() * stateMat.transpose() + m_procNoise;
    }

    return nextState;
}

void ExtendedKalman::setStateModel(std::shared_ptr<dynamics::IDynamics> dynObj,
                                   Eigen::MatrixXd procNoise) {
    if (!dynObj) {
        throw exceptions::TypeError("dynObj can not be nullptr");
    }
    if (procNoise.rows() != procNoise.cols()) {
        throw exceptions::BadParams("Process noise must be square");
    }
    if (procNoise.rows() !=
        static_cast<long int>(dynObj->stateNames().size())) {
        throw exceptions::BadParams(
            "Process nosie size does not match they dynamics model "
            "dimension");
    }

    m_dynObj = dynObj;
    m_procNoise = procNoise;
}

void ExtendedKalman::setMeasurementModel(
    std::shared_ptr<measurements::IMeasModel> measObj,
    Eigen::MatrixXd measNoise) {
    if (!measObj || !utilities:: instanceof
        <measurements::ILinearMeasModel>(measObj) || !utilities:: instanceof
        <measurements::INonLinearMeasModel>(measObj)) {
        throw exceptions::TypeError(
            "measObj must be a derived class of ILinearMeasModel or "
            "INonLinearMeasModel");
    }

    if (measNoise.rows() != measNoise.cols()) {
        throw exceptions::BadParams("Measurement noise must be square");
    }

    if (utilities :: instanceof <measurements::ILinearMeasModel>(measObj)) {
        measObj =
            std::dynamic_pointer_cast<measurements::ILinearMeasModel>(measObj);
        measNoise = measNoise;
    }

    if (utilities :: instanceof <measurements::INonLinearMeasModel>(measObj)) {
        measObj = std::dynamic_pointer_cast<measurements::INonLinearMeasModel>(
            measObj);
        measNoise = measNoise;
    }
}

inline std::shared_ptr<dynamics::IDynamics> ExtendedKalman::dynamicsModel()
    const {
    if (m_dynObj) {
        return m_dynObj;
    } else {
        throw exceptions::TypeError("Dynamics model is unset");
    }
}

}  // namespace lager::gncpy::filters
