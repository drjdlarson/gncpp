#pragma once
#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <functional>
#include <memory>

#include "gncpy/Exceptions.h"
#include "gncpy/SerializeMacros.h"
#include "gncpy/control/ILinearControlModel.h"
#include "gncpy/control/Parameters.h"
#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/dynamics/Parameters.h"

namespace lager::gncpy::dynamics {

/// @brief  Interface for all linear dynamics models
class ILinearDynamics : public IDynamics {
    friend class boost::serialization::access;

   public:
    /**
     * @brief Get the State matrix
     *
     * This is the main function child classes will need to implement as it
     * defines the dynamics for the class. This is allowed to be time varying
     * and depend on external parameters. These parameters may be subclassed to
     * allow specific values per dynamics type. This returns the A matrix from
     * the following equation.
     *
     * \f[
     *      x_{k+1} = F(t) x_k + G(t) u_k
     * \f]
     *
     *
     * @param timestep current timestep
     * @param stateTransParams parameters relevant to the specific model
     * @return Eigen::MatrixXd State transition matrix
     */
    virtual Eigen::MatrixXd getStateMat(
        double timestep,
        const StateTransParams* stateTransParams = nullptr) const = 0;

    Eigen::VectorXd propagateState(
        double timestep, const Eigen::VectorXd& state,
        const StateTransParams* stateTransParams = nullptr) const override;
    Eigen::VectorXd propagateState(double timestep,
                                   const Eigen::VectorXd& state,
                                   const Eigen::VectorXd& control,
                                   const lager::gncpy::control::ControlParams*
                                       controlParams) const override;
    Eigen::VectorXd propagateState(
        double timestep, const Eigen::VectorXd& state,
        const Eigen::VectorXd& control,
        const StateTransParams* stateTransParams,
        const lager::gncpy::control::ControlParams* controlParams,
        const ConstraintParams* constraintParams) const final;

    /**
     * @brief Get the Input/control matrix
     *
     * This returns the B matrix from the following equation, which may be time
     * varying and depend on external parameters. The parameters can be
     * subclassed based on the control model used.
     *
     * \f[
     *      x_{k+1} = A(t) x_k + B(t) u_k
     * \f]
     *
     * @param timestep
     * @param controlParams
     * @return Eigen::MatrixXd
     */

    void setControlModel(
        std::shared_ptr<control::ILinearControlModel>
            model);  // comparable to set dynamics model, use that as a template
    void clearControlModel() override { m_controlModel.reset(); }
    bool hasControlModel() const override {
        return static_cast<bool>(m_controlModel);
    }

    std::shared_ptr<lager::gncpy::control::ILinearControlModel> controlModel()
        const {
        return m_controlModel;
    }

   protected:
    std::shared_ptr<control::ILinearControlModel> m_controlModel;
    Eigen::VectorXd propagateState_(
        double timestep, const Eigen::VectorXd& state,
        const StateTransParams* stateTransParams = nullptr) const;

   private:
    template <class Archive>
    void serialize(Archive& ar, [[maybe_unused]] const unsigned int version) {
        ar& boost::serialization::base_object<IDynamics>(*this);
        // ar& m_controlModel;
    }
};

}  // namespace lager::gncpy::dynamics
