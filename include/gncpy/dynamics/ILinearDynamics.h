/// @file Interface for all linear dynamics models
#pragma once
#include <functional>

#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/dynamics/Parameters.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"

namespace lager::gncpy::dynamics {

/// @brief  Interface for all linear dynamics models
template <typename T>
class ILinearDynamics : public IDynamics<T> {
    friend class cereal::access;

   public:
    virtual ~ILinearDynamics() = default;

    /**
     * @brief Get the State matrix
     *
     * This is the main function child classes will need to implement as it
     * defines the dynamics for the class. This is allowed to be time varying
     * and depend on external parameters. These parameters may be subclassed to
     * allow specific values per dynamics type. This returns the A matrix from
     * the following equation.
     *
     * \f{
     * x_{k+1} = A(t) x_k + B(t) u_k
     * \f}
     *
     *
     * @param timestep current timestep
     * @param stateTransParams parameters relevant to the specific model
     * @return matrix::Matrix<T> State transition matrix
     */
    virtual matrix::Matrix<T> getStateMat(
        T timestep,
        const StateTransParams* stateTransParams = nullptr) const = 0;

    matrix::Vector<T> propagateState(
        T timestep, const matrix::Vector<T>& state,
        const StateTransParams* stateTransParams = nullptr) const override {
        matrix::Vector<T> nextState =
            this->propagateState_(timestep, state, stateTransParams);

        if (this->hasStateConstraint()) {
            this->stateConstraint(timestep, nextState);
        }

        return nextState;
    }

    matrix::Vector<T> propagateState(
        T timestep, const matrix::Vector<T>& state,
        const matrix::Vector<T>& control) const override {
        matrix::Vector<T> nextState = this->propagateState_(timestep, state);

        if (this->hasControlModel()) {
            nextState += this->getInputMat(timestep) * control;
        }

        if (this->hasStateConstraint()) {
            this->stateConstraint(timestep, nextState);
        }

        return nextState;
    }

    matrix::Vector<T> propagateState(
        T timestep, const matrix::Vector<T>& state,
        const matrix::Vector<T>& control,
        const StateTransParams* stateTransParams,
        const ControlParams* controlParams,
        const ConstraintParams* constraintParams) const final {
        matrix::Vector<T> nextState =
            this->propagateState_(timestep, state, stateTransParams);

        if (this->hasControlModel()) {
            nextState += this->getInputMat(timestep, controlParams) * control;
        }

        if (this->hasStateConstraint()) {
            this->stateConstraint(timestep, nextState, constraintParams);
        }

        return nextState;
    }

    /**
     * @brief Get the Input/control matrix
     *
     * This returns the B matrix from the following equation, which may be time
     * varying and depend on external parameters. The parameters can be
     * subclassed based on the control model used.
     *
     * \f{
     * x_{k+1} = A(t) x_k + B(t) u_k
     * \f}
     *
     * @param timestep
     * @param controlParams
     * @return matrix::Matrix<T>
     */
    matrix::Matrix<T> getInputMat(
        T timestep, const ControlParams* controlParams = nullptr) const {
        return controlParams == nullptr
                   ? this->controlModel(timestep)
                   : this->controlModel(timestep, controlParams);
    }

    template <typename F>
    inline void setControlModel(F&& model) {
        m_hasContolModel = true;
        m_controlModel = std::forward<F>(model);
    }
    inline void clearControlModel() override { m_hasContolModel = false; }
    inline bool hasControlModel() const override { return m_hasContolModel; }

    inline std::function<matrix::Matrix<T>(T timestep,
                                           const ControlParams* controlParams)>
    controlModel() const {
        return m_controlModel;
    }

   protected:
    // NOTE: can not serialize std::function or lambda function
    // see
    // https://stackoverflow.com/questions/57095837/serialize-lambda-functions-with-cereal
    template <class Archive>
    void serialize(Archive& ar) {
        bool tmp = m_hasContolModel;
        m_hasContolModel = false;
        ar(cereal::make_nvp("IDynamics",
                            cereal::virtual_base_class<IDynamics<T>>(this)),
           CEREAL_NVP(m_hasContolModel));
        m_hasContolModel = tmp;
    }

    inline matrix::Matrix<T> controlModel(
        T timestep, const ControlParams* controlParams = nullptr) const {
        if (m_hasContolModel) {
            return m_controlModel(timestep, controlParams);
        }
        throw NoControlError();
    }

    inline matrix::Vector<T> propagateState_(
        T timestep, const matrix::Vector<T>& state,
        const StateTransParams* stateTransParams = nullptr) const {
        return stateTransParams == nullptr
                   ? this->getStateMat(timestep) * state
                   : this->getStateMat(timestep, stateTransParams) * state;
    }

   private:
    bool m_hasContolModel = false;
    std::function<matrix::Matrix<T>(T timestep,
                                    const ControlParams* controlParams)>
        m_controlModel;
};

}  // namespace lager::gncpy::dynamics

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::dynamics::ILinearDynamics)
