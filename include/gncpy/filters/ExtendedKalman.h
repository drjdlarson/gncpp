#pragma once
#include <cereal/access.hpp>

#include "gncpy/SerializeMacros.h"
#include "gncpy/Utilities.h"
#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/dynamics/ILinearDynamics.h"
#include "gncpy/dynamics/INonLinearDynamics.h"
#include "gncpy/filters/IBayesFilter.h"
#include "gncpy/filters/Parameters.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"
#include "gncpy/measurements/ILinearMeasModel.h"
#include "gncpy/measurements/IMeasModel.h"
#include "gncpy/measurements/INonLinearMeasModel.h"

namespace lager::gncpy::filters {

template <typename T>
class ExtendedKalman final : public IBayesFilter<T> {
    friend cereal::access;

    GNCPY_SERIALIZE_CLASS(ExtendedKalman<T>)

   public:
    matrix::Vector<T> predict(
        T timestep, const matrix::Vector<T>& curState,
        const std::optional<matrix::Vector<T>> controlInput,
        const BayesPredictParams* params = nullptr) override;

    /**
     * @brief Implements a discrete time correction step
     *
     * @param timestep current timestep
     * @param meas current measurement
     * @param curState current state estimate
     * @param measFitProb ouptut measurement fit probability
     * @param params correction step parameters
     * @return matrix::Vector<T> corrected state estimate
     */
    matrix::Vector<T> correct(
        [[maybe_unused]] T timestep, const matrix::Vector<T>& meas,
        const matrix::Vector<T>& curState, T& measFitProb,
        const BayesCorrectParams* params = nullptr) override;

    void setStateModel(std::shared_ptr<dynamics::IDynamics<T>> dynObj,
                       matrix::Matrix<T> procNoise) override;
    void setMeasurementModel(
        std::shared_ptr<measurements::IMeasModel<T>> measObj,
        matrix::Matrix<T> measNoise) override;

    std::shared_ptr<dynamics::IDynamics<T>> dynamicsModel() const override;
    std::shared_ptr<measurements::IMeasModel<T>> measurementModel()
        const override;

   private:
    template <class Archive>
    void serialize(Archive& ar);

    bool m_continuousCov = false;

    matrix::Matrix<T> m_measNoise;
    matrix::Matrix<T> m_procNoise;

    std::shared_ptr<dynamics::IDynamics<T>> m_dynObj;
    std::shared_ptr<measurements::IMeasModel<T>> m_measObj;
};

template <typename T>
matrix::Vector<T> ExtendedKalman<T>::predict(
    T timestep, const matrix::Vector<T>& curState,
    [[maybe_unused]] const std::optional<matrix::Vector<T>> controlInput,
    const BayesPredictParams* params) {
    if (params != nullptr && !utilities:: instanceof
        <BayesPredictParams>(params)) {
        throw exceptions::BadParams("Params must be BayesPredictParams");
    }

    matrix::Vector<T> nextState = this->m_dynObj->propagateState(
        timestep, curState, params->stateTransParams.get());

    matrix::Matrix<T> stateMat;
    if (utilities:: instanceof
        <dynamics::INonLinearDynamics<T>>(this->dynamicsModel())) {
        stateMat = std::dynamic_pointer_cast<dynamics::INonLinearDynamics<T>>(
                       this->dynamicsModel())
                       ->getStateMat(timestep, curState,
                                     params->stateTransParams.get());
    } else if (utilities:: instanceof
               <dynamics::ILinearDynamics<T>>(this->dynamicsModel())) {
        stateMat = std::dynamic_pointer_cast<dynamics::ILinearDynamics<T>>(
                       this->dynamicsModel())
                       ->getStateMat(timestep, params->stateTransParams.get());
    } else {
        throw exceptions::TypeError("Unknown dynamics type");
    }

    if (m_continuousCov) {
        throw std::runtime_error(
            "Continuous covariance model not implemented yet for the EKF");
    } else {
        this->cov =
            stateMat * this->cov * stateMat.transpose() + this->m_procNoise;
    }

    return nextState;
}

template <typename T>
matrix::Vector<T> ExtendedKalman<T>::correct([[maybe_unused]] T timestep,
                                             const matrix::Vector<T>& meas,
                                             const matrix::Vector<T>& curState,
                                             T& measFitProb,
                                             const BayesCorrectParams* params) {
    if (params != nullptr && !utilities:: instanceof
        <BayesCorrectParams>(params)) {
        throw exceptions::BadParams("Params must be BayesCorrectParams");
    }

    matrix::Vector<T> estMeas =
        this->measurementModel()->measure(curState, params->measParams.get());
    matrix::Matrix<T> measMat = this->measurementModel()->getMeasMat(
        curState, params->measParams.get());

    matrix::Matrix<T> inovCov = measMat * this->cov * measMat.transpose();

    matrix::Matrix<T> kalmanGain =
        this->cov * measMat.transpose() * inovCov.inverse();

    matrix::Vector<T> inov = meas - estMeas;
    this->cov -= kalmanGain * measMat * this->cov;

    measFitProb = math::calcGaussianPDF(meas, estMeas, inovCov);

    return curState + kalmanGain * inov;
}

template <typename T>
void ExtendedKalman<T>::setStateModel(
    std::shared_ptr<dynamics::IDynamics<T>> dynObj,
    matrix::Matrix<T> procNoise) {
    if (!dynObj) {
        throw exceptions::TypeError("dynObj can not be nullptr");
    }
    if (procNoise.numRows() != procNoise.numCols()) {
        throw exceptions::BadParams("Process noise must be square");
    }
    if (procNoise.numRows() != dynObj->stateNames().size()) {
        throw exceptions::BadParams(
            "Process nosie size does not match they dynamics model "
            "dimension");
    }

    this->m_dynObj = dynObj;
    this->m_procNoise = procNoise;
}

template <typename T>
void ExtendedKalman<T>::setMeasurementModel(
    std::shared_ptr<measurements::IMeasModel<T>> measObj,
    matrix::Matrix<T> measNoise) {
    if (!measObj) {
        throw exceptions::TypeError("measObj can not be nullptr");
    }

    if (measNoise.numRows() != measNoise.numCols()) {
        throw exceptions::BadParams("Measurement noise must be squqre");
    }

    this->m_measObj = measObj;
    this->m_measNoise = measNoise;
}

template <typename T>
inline std::shared_ptr<dynamics::IDynamics<T>>
ExtendedKalman<T>::dynamicsModel() const {
    if (m_dynObj) {
        return m_dynObj;
    } else {
        throw exceptions::TypeError("Dynamics model is unset");
    }
}

template <typename T>
inline std::shared_ptr<measurements::IMeasModel<T>>
ExtendedKalman<T>::measurementModel() const {
    if (m_measObj) {
        return m_measObj;
    } else {
        throw exceptions::TypeError("Measurement model is unset");
    }
}

template <typename T>
template <class Archive>
void ExtendedKalman<T>::serialize(Archive& ar) {
    ar(cereal::make_nvp("Kalman",
                        cereal::virtual_base_class<IBayesFilter<T>>(this)),
       CEREAL_NVP(m_measNoise), CEREAL_NVP(m_procNoise), CEREAL_NVP(m_dynObj),
       CEREAL_NVP(m_measObj), CEREAL_NVP(m_continuousCov));
}

extern template class ExtendedKalman<float>;
extern template class ExtendedKalman<double>;

}  // namespace lager::gncpy::filters

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::filters::ExtendedKalman)
