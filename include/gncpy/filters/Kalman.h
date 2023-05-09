#pragma once
#include <memory>
#include <optional>

#include "gncpy/Exceptions.h"
#include "gncpy/SerializeMacros.h"
#include "gncpy/Utilities.h"
#include "gncpy/dynamics/SerializableTypes.h"
#include "gncpy/filters/IBayesFilter.h"
#include "gncpy/filters/Parameters.h"
#include "gncpy/math/Math.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"
#include "gncpy/measurements/SerializableTypes.h"

namespace lager::gncpy::filters {

/**
 * @brief Implements a Kalman Filter
 *
 * This is a discrete time KF loosely based on
 * @cite Crassidis2011_OptimalEstimationofDynamicSystems
 *
 * @tparam T
 */
template <typename T>
class Kalman : public IBayesFilter<T> {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(Kalman<T>)

   public:
    /**
     * @brief Implements a discrete time prediction step
     *
     * @param timestep current timestep
     * @param curState current state estmiate
     * @param controlInput optional control input
     * @param params prediction parameters
     * @return matrix::Vector<T> predicted state estimate
     */
    matrix::Vector<T> predict(
        T timestep, const matrix::Vector<T>& curState,
        [[maybe_unused]] const std::optional<matrix::Vector<T>> controlInput,
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

    /**
     * @brief Sets the state model equation for the filter.
     *
     * The dynamics must be linear and may be constant or time varying. This
     * assumes a discrete model of the form
     *
     * \f[
     *      x_{k+1} = F(t) x_k + G(t) u_k
     * \f]
     *
     * @param dynObj Linear dynamics of type dynamics::ILinearDynamics
     * @param procNoise Process noise matrix for the filter
     */
    void setStateModel(std::shared_ptr<dynamics::IDynamics<T>> dynObj,
                       matrix::Matrix<T> procNoise) override;

    /**
     * @brief Sets the measurement model for the filter.
     *
     * This can either set the constant measurement matrix, or the matrix can be
     * time varying. This assumes a measurement model of the form
     *
     * \f[
     *      \tilde{y}_{k+1} = H(t) x_{k+1}^-
     * \f]
     *
     * @param measObj linear measurement model of type
     * measurements::ILinearMeasModel
     * @param measNoise measurement nosie matrix for the filter
     */
    void setMeasurementModel(
        std::shared_ptr<measurements::IMeasModel<T>> measObj,
        matrix::Matrix<T> measNoise) override;

    std::shared_ptr<dynamics::IDynamics<T>> dynamicsModel() const override;
    std::shared_ptr<measurements::IMeasModel<T>> measurementModel()
        const override;

   private:
    template <class Archive>
    void serialize(Archive& ar);

    matrix::Matrix<T> m_measNoise;
    matrix::Matrix<T> m_procNoise;

    std::shared_ptr<dynamics::ILinearDynamics<T>> m_dynObj;
    std::shared_ptr<measurements::ILinearMeasModel<T>> m_measObj;
};

template <typename T>
matrix::Vector<T> Kalman<T>::predict(
    T timestep, const matrix::Vector<T>& curState,
    [[maybe_unused]] const std::optional<matrix::Vector<T>> controlInput,
    const BayesPredictParams* params) {
    if (params != nullptr && !utilities:: instanceof
        <BayesPredictParams>(params)) {
        throw exceptions::BadParams("Params must be BayesPredictParams");
    }
    matrix::Matrix<T> stateMat =
        this->m_dynObj->getStateMat(timestep, params->stateTransParams.get());
    this->cov = stateMat * this->cov * stateMat.transpose() + this->m_procNoise;

    return this->m_dynObj->propagateState(timestep, curState,
                                          params->stateTransParams.get());
}

template <typename T>
matrix::Vector<T> Kalman<T>::correct([[maybe_unused]] T timestep,
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
void Kalman<T>::setStateModel(std::shared_ptr<dynamics::IDynamics<T>> dynObj,
                              matrix::Matrix<T> procNoise) {
    if (!dynObj || !utilities:: instanceof
        <dynamics::ILinearDynamics<T>>(dynObj)) {
        throw exceptions::TypeError(
            "dynObj must be a derived class of ILinearDynamics");
    }
    if (procNoise.numRows() != procNoise.numCols()) {
        throw exceptions::BadParams("Process noise must be square");
    }
    if (procNoise.numRows() != dynObj->stateNames().size()) {
        throw exceptions::BadParams(
            "Process nosie size does not match they dynamics model "
            "dimension");
    }

    this->m_dynObj =
        std::dynamic_pointer_cast<dynamics::ILinearDynamics<T>>(dynObj);
    this->m_procNoise = procNoise;
}

template <typename T>
void Kalman<T>::setMeasurementModel(
    std::shared_ptr<measurements::IMeasModel<T>> measObj,
    matrix::Matrix<T> measNoise) {
    if (!measObj || !utilities:: instanceof
        <measurements::ILinearMeasModel<T>>(measObj)) {
        throw exceptions::TypeError(
            "measObj must be a derived class of ILinearMeasModel");
    }

    if (measNoise.numRows() != measNoise.numCols()) {
        throw exceptions::BadParams("Measurement noise must be squqre");
    }

    this->m_measObj =
        std::dynamic_pointer_cast<measurements::ILinearMeasModel<T>>(measObj);
    this->m_measNoise = measNoise;
}

template <typename T>
inline std::shared_ptr<dynamics::IDynamics<T>> Kalman<T>::dynamicsModel()
    const {
    if (m_dynObj) {
        return m_dynObj;
    } else {
        throw exceptions::TypeError("Dynamics model is unset");
    }
}

template <typename T>
inline std::shared_ptr<measurements::IMeasModel<T>>
Kalman<T>::measurementModel() const {
    if (m_measObj) {
        return m_measObj;
    } else {
        throw exceptions::TypeError("Measurement model is unset");
    }
}

template <typename T>
template <class Archive>
void Kalman<T>::serialize(Archive& ar) {
    ar(cereal::make_nvp("IBayesFilter",
                        cereal::virtual_base_class<IBayesFilter<T>>(this)),
       CEREAL_NVP(m_measNoise), CEREAL_NVP(m_procNoise), CEREAL_NVP(m_dynObj),
       CEREAL_NVP(m_measObj));
}

extern template class Kalman<float>;
extern template class Kalman<double>;

}  // namespace lager::gncpy::filters

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::filters::Kalman)
