#include "gncpy/filters/Kalman.h"

#include "gncpy/Exceptions.h"
#include "gncpy/Utilities.h"

namespace lager::gncpy::filters {

Eigen::VectorXd Kalman::predict(
    double timestep, const Eigen::VectorXd& curState,
    [[maybe_unused]] const std::optional<Eigen::VectorXd> controlInput,
    const BayesPredictParams* params) {
    if (params != nullptr && !utilities:: instanceof
        <BayesPredictParams>(params)) {
        throw exceptions::BadParams("Params must be BayesPredictParams");
    }
    Eigen::MatrixXd stateMat =
        m_dynObj->getStateMat(timestep, params->stateTransParams.get());
    getCov() = stateMat * viewCov() * stateMat.transpose() + m_procNoise;

    return m_dynObj->propagateState(timestep, curState,
                                    params->stateTransParams.get());
}

Eigen::VectorXd Kalman::correct([[maybe_unused]] double timestep,
                                const Eigen::VectorXd& meas,
                                const Eigen::VectorXd& curState,
                                double& measFitProb,
                                const BayesCorrectParams* params) {
    if (params != nullptr && !utilities:: instanceof
        <BayesCorrectParams>(params)) {
        throw exceptions::BadParams("Params must be BayesCorrectParams");
    }

    Eigen::VectorXd estMeas =
        measurementModel()->measure(curState, params->measParams.get());
    Eigen::MatrixXd measMat =
        measurementModel()->getMeasMat(curState, params->measParams.get());

    Eigen::MatrixXd inovCov = measMat * viewCov() * measMat.transpose();

    Eigen::MatrixXd kalmanGain =
        viewCov() * measMat.transpose() * inovCov.inverse();

    Eigen::VectorXd inov = meas - estMeas;
    getCov() -= kalmanGain * measMat * viewCov();

    measFitProb = math::calcGaussianPDF(meas, estMeas, inovCov);

    return curState + kalmanGain * inov;
}

void Kalman::setStateModel(std::shared_ptr<dynamics::IDynamics> dynObj,
                           Eigen::MatrixXd procNoise) {
    if (!dynObj || !utilities:: instanceof
        <dynamics::ILinearDynamics>(dynObj)) {
        throw exceptions::TypeError(
            "dynObj must be a derived class of ILinearDynamics");
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

    m_dynObj = std::dynamic_pointer_cast<dynamics::ILinearDynamics>(dynObj);
    m_procNoise = procNoise;
}

void Kalman::setMeasurementModel(
    std::shared_ptr<measurements::IMeasModel> measObj,
    Eigen::MatrixXd measNoise) {
    if (!measObj || !utilities:: instanceof
        <measurements::ILinearMeasModel>(measObj)) {
        throw exceptions::TypeError(
            "measObj must be a derived class of ILinearMeasModel");
    }

    if (measNoise.rows() != measNoise.cols()) {
        throw exceptions::BadParams("Measurement noise must be squqre");
    }

    m_measObj =
        std::dynamic_pointer_cast<measurements::ILinearMeasModel>(measObj);
    m_measNoise = measNoise;
}

inline std::shared_ptr<dynamics::IDynamics> Kalman::dynamicsModel() const {
    if (m_dynObj) {
        return m_dynObj;
    } else {
        throw exceptions::TypeError("Dynamics model is unset");
    }
}

inline std::shared_ptr<measurements::IMeasModel> Kalman::measurementModel()
    const {
    if (m_measObj) {
        return m_measObj;
    } else {
        throw exceptions::TypeError("Measurement model is unset");
    }
}

}  // namespace lager::gncpy::filters
