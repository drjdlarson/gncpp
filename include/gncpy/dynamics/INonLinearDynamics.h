#pragma once
#include <functional>

#include "gncpy/Exceptions.h"
#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/dynamics/Parameters.h"
#include "gncpy/math/Math.h"
#include "gncpy/math/Vector.h"

namespace lager::gncpy::dynamics {

/// @brief Interface for all non-linear dynamics models
/// @todo Finish implementing this class
template <typename T>
class INonLinearDynamics : public IDynamics<T> {
    friend class cereal::access;

   public:
    virtual ~INonLinearDynamics() = default;

    virtual matrix::Vector<T> continuousDynamics(
        T timestep, const matrix::Vector<T>& state,
        const StateTransParams* stateTransParams = nullptr) const = 0;
    matrix::Vector<T> propagateState(
        T timestep, const matrix::Vector<T>& state,
        const StateTransParams* stateTransParams = nullptr) const override;
    matrix::Vector<T> propagateState(
        T timestep, const matrix::Vector<T>& state,
        const matrix::Vector<T>& control) const override;
    matrix::Vector<T> propagateState(
        T timestep, const matrix::Vector<T>& state,
        const matrix::Vector<T>& control,
        const StateTransParams* const stateTransParams,
        const ControlParams* const controlParams,
        const ConstraintParams* const constraintParams) const final;

    template <typename F>
    void setControlModel(F&& model, bool continuousModel);
    void clearControlModel() override { m_hasControlModel = false; }
    bool hasControlModel() const override { return m_hasControlModel; }

    matrix::Matrix<T> getStateMat(
        T timestep, const matrix::Vector<T>& state,
        const StateTransParams* stateTransParams = nullptr) const;

    T dt() const { return m_dt; }
    void setDt(T dt) { m_dt = dt; }

   private:
    // NOTE: can not serialize std::function or lambda function
    // see
    // https://stackoverflow.com/questions/57095837/serialize-lambda-functions-with-cereal
    template <class Archive>
    void serialize(Archive& ar);

    T m_dt = 0;
    bool m_hasControlModel = false;
    bool m_continuousControl = false;
    std::function<matrix::Vector<T>(T timestep, const matrix::Vector<T>& state,
                                    const matrix::Vector<T>& control,
                                    const ControlParams* controlParams)>
        m_controlModel;
};

template <typename T>
matrix::Vector<T> INonLinearDynamics<T>::propagateState(
    T timestep, const matrix::Vector<T>& state,
    const StateTransParams* stateTransParams) const {
    matrix::Vector<T> nextState =
        math::rungeKutta4<matrix::Vector<T>, matrix::Vector<T>, T>(
            timestep, state, this->dt(),
            [this, stateTransParams](T t, const matrix::Vector<T>& x) {
                return this->continuousDynamics(t, x, stateTransParams);
            });

    if (this->hasStateConstraint()) {
        this->stateConstraint(timestep, nextState);
    }

    return nextState;
}

template <typename T>
matrix::Vector<T> INonLinearDynamics<T>::propagateState(
    T timestep, const matrix::Vector<T>& state,
    const matrix::Vector<T>& control) const {
    matrix::Vector<T> nextState;
    if (m_hasControlModel && m_continuousControl) {
        nextState = math::rungeKutta4<matrix::Vector<T>, matrix::Vector<T>, T>(
            timestep, state, this->dt(),
            [this, &control](T t, const matrix::Vector<T>& x) {
                return this->continuousDynamics(t, x) +
                       this->m_controlModel(t, x, control, nullptr);
            });
    } else if (m_hasControlModel) {
        nextState = math::rungeKutta4<matrix::Vector<T>, matrix::Vector<T>, T>(
            timestep, state, this->dt(),
            [this](T t, const matrix::Vector<T>& x) {
                return this->continuousDynamics(t, x);
            });

        nextState += this->m_controlModel(timestep, state, control, nullptr);
    } else {
        throw exceptions::BadParams(
            "Control input given but no control model set");
    }

    if (this->hasStateConstraint()) {
        this->stateConstraint(timestep, nextState);
    }

    return nextState;
}

template <typename T>
matrix::Vector<T> INonLinearDynamics<T>::propagateState(
    T timestep, const matrix::Vector<T>& state,
    const matrix::Vector<T>& control,
    const StateTransParams* const stateTransParams,
    const ControlParams* const controlParams,
    const ConstraintParams* const constraintParams) const {
    matrix::Vector<T> nextState;
    if (m_hasControlModel && m_continuousControl) {
        nextState = math::rungeKutta4<matrix::Vector<T>, matrix::Vector<T>, T>(
            timestep, state, this->dt(),
            [this, &control, stateTransParams, controlParams](
                T t, const matrix::Vector<T>& x) {
                return this->continuousDynamics(t, x, stateTransParams) +
                       this->m_controlModel(t, x, control, controlParams);
            });
    } else if (m_hasControlModel) {
        nextState = math::rungeKutta4<matrix::Vector<T>, matrix::Vector<T>, T>(
            timestep, state, this->dt(),
            [this](T t, const matrix::Vector<T>& x) {
                return this->continuousDynamics(t, x);
            });

        nextState +=
            this->m_controlModel(timestep, state, control, controlParams);
    } else {
        nextState = math::rungeKutta4<matrix::Vector<T>, matrix::Vector<T>, T>(
            timestep, state, this->dt(),
            [this, stateTransParams](T t, const matrix::Vector<T>& x) {
                return this->continuousDynamics(t, x, stateTransParams);
            });
    }

    if (this->hasStateConstraint()) {
        this->stateConstraint(timestep, nextState, constraintParams);
    }

    return nextState;
}

template <typename T>
template <typename F>
void INonLinearDynamics<T>::setControlModel(F&& model, bool continuousModel) {
    m_hasControlModel = true;
    m_continuousControl = continuousModel;
    m_controlModel = std::forward<F>(model);
}

template <typename T>
matrix::Matrix<T> INonLinearDynamics<T>::getStateMat(
    T timestep, const matrix::Vector<T>& state,
    const StateTransParams* stateTransParams) const {
    return math::getJacobian<T>(
        state,
        [this, timestep, stateTransParams](const matrix::Vector<T>& x) {
            return this->continuousDynamics(timestep, x, stateTransParams);
        },
        state.size());
}

template <typename T>
template <class Archive>
void INonLinearDynamics<T>::serialize(Archive& ar) {
    bool tmp = m_hasControlModel;
    m_hasControlModel = false;
    ar(cereal::make_nvp("IDynamics",
                        cereal::virtual_base_class<IDynamics<T>>(this)),
       CEREAL_NVP(m_dt), CEREAL_NVP(m_hasControlModel),
       CEREAL_NVP(m_continuousControl));
    m_hasControlModel = tmp;
}

}  // namespace lager::gncpy::dynamics

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::dynamics::INonLinearDynamics)
