#pragma once
#include <cereal/access.h>

#include "gncpy/SerializeMacros.h"
#include "gncpy/Utilites.h"
#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/filters/IBayesFilter.h"
#include "gncpy/filters/Parameters.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"
#include "gncpy/measurements/IMeasModel.h"

namespace lager::gncpy::filters {

template <typename T>
class ExtendedKalman final : public IBayesFilter<T> {
    friend cereal::access;

   public:
    matrix::Vector<T> predict(
        T timestep, const matrix::Vector<T>& curState,
        const std::optional<matrix::Vector<T>> controlInput,
        const BayesPredictParams* params = nullptr) override;
    matrix::Vector<T> correct(
        T timestep, const matrix::Vector<T>& meas,
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
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("IBayesFilter",
                            cereal::virtual_base_class<IBayesFilter<T>>(this)),
           CEREAL_NVP(m_measNoise), CEREAL_NVP(m_procNoise),
           CEREAL_NVP(m_dynObj), CEREAL_NVP(m_measObj));
    }

    matrix::Matrix<T> m_measNoise;
    matrix::Matrix<T> m_procNoise;

    std::shared_ptr<dynamics::IDynamics<T>> m_dynObj;
    std::shared_ptr<measurements::IMeasModel<T>> m_measObj;
};

template <typename T>
matrix::Vector<T> ExtendedKalman<T> predict(
    T timestep, const matrix::Vector<T>& curState,
    const std::optional<matrix::Vector<T>> controlInput,
    const BayesPredictParams* params = nullptr) {
    matrix::Vector<T> nextState = m_dynobj->propagateState(
        timestep, curState, controlInput, params->stateTransParams,
        params->controlParams, nullptr);
}

}  // namespace lager::gncpy::filters
