#pragma once
#include <Eigen/Dense>
#include <functional>

#include "gncpy/Exceptions.h"
#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/dynamics/Parameters.h"

#include "gncpy/control/Parameters.h"
#include "gncpy/control/IControlModel.h"

namespace lager::gncpy::dynamics {

/// @brief  Interface for all linear dynamics models
class ILinearDynamics : public IDynamics {
    friend class cereal::access;

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
    Eigen::VectorXd propagateState(
        double timestep, const Eigen::VectorXd& state,
        const Eigen::VectorXd& control,
        const lager::gncpy::control::ControlParams* controlParams) const override;
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

    // template <typename F>
    // void setControlModel(F&& model);
    void setControlModel(std::shared_ptr<control::IControlModel> model); // comparable to set dynamics model, use that as a template
    inline void clearControlModel() override { m_hasContolModel = false; }
    inline bool hasControlModel() const override { return m_hasContolModel; }

    inline std::shared_ptr<lager::gncpy::control::IControlModel> controlModel() const {return m_controlModel; }
    // inline std::function<Eigen::MatrixXd(double timestep,
    //                                      const lager::gncpy::control::ControlParams* controlParams)>
    // controlModel() const {
    //     return m_controlModel;
    // }

   protected:
    // NOTE: can not serialize std::function or lambda function
    // see
    // https://stackoverflow.com/questions/57095837/serialize-lambda-functions-with-cereal
    std::shared_ptr<control::IControlModel> m_controlModel;
    template <class Archive>
    void serialize(Archive& ar);

    // Eigen::MatrixXd controlModel(
    //     double timestep, const ControlParams* controlParams = nullptr) const;


    Eigen::VectorXd propagateState_(
        double timestep, const Eigen::VectorXd& state,
        const StateTransParams* stateTransParams = nullptr) const;

   private:
    bool m_hasContolModel = false;

    
};

// template <typename F>
// void ILinearDynamics::setControlModel(std::shared_ptr<control::IControlModel> model) {
//     m_hasContolModel = true;
//     m_controlModel = model;
// }

template <class Archive>
void ILinearDynamics::serialize(Archive& ar) {
    bool tmp = m_hasContolModel;
    m_hasContolModel = false;
    ar(cereal::make_nvp("IDynamics",
                        cereal::virtual_base_class<IDynamics>(this)),
       CEREAL_NVP(m_hasContolModel));
    m_hasContolModel = tmp;
}

}  // namespace lager::gncpy::dynamics

CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::ILinearDynamics)
