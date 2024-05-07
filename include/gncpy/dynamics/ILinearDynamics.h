#pragma once
#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/shared_ptr.hpp>
#include <functional>
#include <memory>

#include "gncpy/Exceptions.h"
#include "gncpy/SerializeHelper.h"
#include "gncpy/SerializeMacros.h"
#include "gncpy/control/ILinearControlModel.h"
#include "gncpy/control/Parameters.h"
// #include "gncpy/control/SerializeSubclass.h"
#include "gncpy/control/StateControl.h"
#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/dynamics/Parameters.h"

// BOOST_SERIALIZATION_SPLIT_FREE(
//     boost::shared_ptr<lager::gncpy::control::ILinearControlModel>)

// namespace boost {
// namespace serialization {
// template <class Archive>
// void save(
//     Archive& ar,
//     const boost::shared_ptr<lager::gncpy::control::ILinearControlModel>*
//     cMdl,
//     [[maybe_unused]] const unsigned int version) {
//     // invoke serialization of the base class
//     ar << cMdl;
// }

// template <class Archive>
// void load(Archive& ar,
//           boost::shared_ptr<lager::gncpy::control::ILinearControlModel>*
//           cMdl,
//           [[maybe_unused]] const unsigned int version) {
//     // invoke serialization of the base class
//     ar >> cMdl;
// }
// }  // namespace serialization
// }  // namespace boost

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

    void setControlModel(
        boost::shared_ptr<control::ILinearControlModel>
            model);  // comparable to set dynamics model, use that as a template
    void clearControlModel() override { m_controlModel.reset(); }

    bool hasControlModel() const override {
        return static_cast<bool>(m_controlModel);
    }

    boost::shared_ptr<lager::gncpy::control::ILinearControlModel> controlModel()
        const {
        if (hasControlModel()) {
            return m_controlModel;
        } else {
            throw NoControlError();
        }
    }

   protected:
    boost::shared_ptr<lager::gncpy::control::ILinearControlModel>
        m_controlModel;
    Eigen::VectorXd propagateState_(
        double timestep, const Eigen::VectorXd& state,
        const StateTransParams* stateTransParams = nullptr) const;

   private:
    template <class Archive>
    void serialize(Archive& ar, [[maybe_unused]] const unsigned int version) {
        // boost::serialization::split_member(ar, *this, version);
        ar& boost::serialization::base_object<IDynamics>(*this);
        ar& this->m_controlModel;
        ar&* m_controlModel;
    }
};

}  // namespace lager::gncpy::dynamics
