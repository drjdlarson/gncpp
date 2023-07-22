#pragma once
#include <Eigen/Dense>
#include <functional>

// #include "gncpy/SerializeMacros.h"
#include "gncpy/control/IControlModel.h"
#include "gncpy/control/Parameters.h"
#include "gncpy/dynamics/Exceptions.h"
#include "gncpy/dynamics/Parameters.h"

namespace lager::gncpy::dynamics {

/// @brief Base interface for all dynamics
class IDynamics {
    friend class boost::serialization::access;

   public:
    virtual ~IDynamics() = default;

    /**
     * @brief Propagate the state forward one timestep
     *
     * @param timestep current time
     * @param state current state
     * @param stateTransParams Parameters needed by the transition model
     * @return Eigen::VectorXd Next state
     */
    virtual Eigen::VectorXd propagateState(
        double timestep, const Eigen::VectorXd& state,
        const StateTransParams* const stateTransParams = nullptr) const = 0;

    /**
     * @brief Propagate the state forward one timestep
     *
     * @param timestep current time step
     * @param state current state
     * @param control control input vector
     * @param controlParams Parameters needed by the control model
     * @return Eigen::VectorXd Next state
     */
    virtual Eigen::VectorXd propagateState(
        double timestep, const Eigen::VectorXd& state,
        const Eigen::VectorXd& control,
        const lager::gncpy::control::ControlParams* controlParams) const = 0;

    /**
     * @brief Propagate the state forward one timestep
     *
     * @param timestep current time step
     * @param state current state
     * @param control control input vector
     * @param stateTransParams Parameters needed by the transition model
     * @param controlParams Parameters needed by the control model
     * @param constraintParams Parameters needed by the constraint model
     * @return Eigen::VectorXd Next state
     */
    virtual Eigen::VectorXd propagateState(
        double timestep, const Eigen::VectorXd& state,
        const Eigen::VectorXd& control,
        const StateTransParams* const stateTransParams,
        const lager::gncpy::control::ControlParams* const controlParams,
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
        double timestep, Eigen::VectorXd& state,
        const ConstraintParams* const constraintParams = nullptr) const;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar& hasStateConstraint;
    }

    bool m_hasStateConstraint = false;
    std::function<void(double timestep, Eigen::VectorXd& state,
                       const ConstraintParams* const constraintParams)>
        m_stateConstraints;
};

template <typename F>
void IDynamics::setStateConstraints(F&& constrants) {
    m_hasStateConstraint = false;
    m_stateConstraints = std::forward<F>(constrants);
}

// template <class Archive>
// void IDynamics::serialize(Archive& ar) {
//     bool tmp = m_hasStateConstraint;
//     m_hasStateConstraint = false;
//     ar(CEREAL_NVP(m_hasStateConstraint));
//     m_hasStateConstraint = tmp;
// }

}  // namespace lager::gncpy::dynamics
