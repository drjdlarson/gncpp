#pragma once
#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>
#include <concepts>
#include <functional>

#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/Exceptions.h"
#include "gncpy/dynamics/Parameters.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"

namespace lager::gncpy::dynamics {

/// @brief Base interface for all dynamics
template <typename T>
    requires std::integral<T> || std::floating_point<T>
class IDynamics {
    friend class cereal::access;

   public:
    virtual ~IDynamics() = default;

    /**
     * @brief Propagate the state forward one timestep
     *
     * @param timestep current time
     * @param state current state
     * @param stateTransParams Parameters needed by the transition model
     * @return matrix::Vector<T> Next state
     */
    virtual matrix::Vector<T> propagateState(
        T timestep, const matrix::Vector<T>& state,
        const StateTransParams* const stateTransParams = nullptr) const = 0;

    /**
     * @brief Propagate the state forward one timestep
     *
     * @param timestep current time step
     * @param state current state
     * @param control control input vector
     * @return matrix::Vector<T> Next state
     */
    virtual matrix::Vector<T> propagateState(
        T timestep, const matrix::Vector<T>& state,
        const matrix::Vector<T>& control) const = 0;

    /**
     * @brief Propagate the state forward one timestep
     *
     * @param timestep current time step
     * @param state current state
     * @param control control input vector
     * @param stateTransParams Parameters needed by the transition model
     * @param controlParams Parameters needed by the control model
     * @param constraintParams Parameters needed by the constraint model
     * @return matrix::Vector<T> Next state
     */
    virtual matrix::Vector<T> propagateState(
        T timestep, const matrix::Vector<T>& state,
        const matrix::Vector<T>& control,
        const StateTransParams* const stateTransParams,
        const ControlParams* const controlParams,
        const ConstraintParams* const constraintParams) const = 0;

    /// @brief Remove the control model
    virtual void clearControlModel() = 0;

    /// @brief Indicates if there is a control model
    virtual bool hasControlModel() const = 0;

    /// @brief Gives a list of the state names, in order
    virtual std::vector<std::string> stateNames() const = 0;

    /**
     * @brief Set the State Constraints object
     *
     * @tparam F
     * @param constrants
     */
    template <typename F>
    void setStateConstraints(F&& constrants);
    inline void clearStateConstraints() { m_hasStateConstraint = false; }
    inline bool hasStateConstraint() const { return m_hasStateConstraint; }

   protected:
    void stateConstraint(
        T timestep, matrix::Vector<T>& state,
        const ConstraintParams* const constraintParams = nullptr) const;

   private:
    template <class Archive>
    void serialize(Archive& ar);

    bool m_hasStateConstraint = false;
    std::function<void(T timestep, matrix::Vector<T>& state,
                       const ConstraintParams* const constraintParams)>
        m_stateConstraints;
};

template <typename T>
template <typename F>
inline void IDynamics<T>::setStateConstraints(F&& constrants) {
    m_hasStateConstraint = false;
    m_stateConstraints = std::forward<F>(constrants);
}

template <typename T>
inline void IDynamics<T>::stateConstraint(
    T timestep, matrix::Vector<T>& state,
    const ConstraintParams* const constraintParams) const {
    if (m_hasStateConstraint) {
        m_stateConstraints(timestep, state, constraintParams);
    }
    throw NoStateConstraintError();
}

template <typename T>
template <class Archive>
void IDynamics<T>::serialize(Archive& ar) {
    bool tmp = m_hasStateConstraint;
    m_hasStateConstraint = false;
    ar(CEREAL_NVP(m_hasStateConstraint));
    m_hasStateConstraint = tmp;
}

}  // namespace lager::gncpy::dynamics

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::dynamics::IDynamics)
