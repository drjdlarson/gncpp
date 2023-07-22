#pragma once
#include <Eigen/Dense>
#include <functional>
#include <memory>

// #include "gncpy/SerializeMacros.h"
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/base_object.hpp>
#include "gncpy/control/IControlModel.h"
#include "gncpy/control/Parameters.h"
#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/dynamics/Parameters.h"

namespace lager::gncpy::dynamics {

/// @brief Interface for all non-linear dynamics models
class INonLinearDynamics : public IDynamics {
    friend class boost::serialization::access;

   public:
    virtual ~INonLinearDynamics() = default;

    virtual Eigen::VectorXd continuousDynamics(
        double timestep, const Eigen::VectorXd& state,
        const StateTransParams* stateTransParams = nullptr) const = 0;
    Eigen::VectorXd propagateState(
        double timestep, const Eigen::VectorXd& state,
        const StateTransParams* stateTransParams = nullptr) const override;
    Eigen::VectorXd propagateState(
        double timestep, const Eigen::VectorXd& state,
        const Eigen::VectorXd& control,
        const control::ControlParams* controlParams) const override;
    Eigen::VectorXd propagateState(
        double timestep, const Eigen::VectorXd& state,
        const Eigen::VectorXd& control,
        const StateTransParams* const stateTransParams,
        const control::ControlParams* const controlParams,
        const ConstraintParams* const constraintParams) const final;

    // template <typename F>
    void setControlModel(std::shared_ptr<control::IControlModel> model,
                         bool continuousModel);
    void clearControlModel() override { m_hasControlModel = false; }
    bool hasControlModel() const override { return m_hasControlModel; }

    Eigen::MatrixXd getStateMat(
        double timestep, const Eigen::VectorXd& state,
        const StateTransParams* stateTransParams = nullptr) const;

    double dt() const { return m_dt; }
    void setDt(double dt) { m_dt = dt; }

   protected:
    std::shared_ptr<control::IControlModel> m_controlModel;

   private:
    // NOTE: can not serialize std::function or lambda function
    // see
    // https://stackoverflow.com/questions/57095837/serialize-lambda-functions-with-cereal
    template <class Archive>
    void serialize(Archive& ar) {
        ar& boost::serialization::base_object<IDynamics>(*this);
        ar& m_dt;
        ar& m_controlModel;
    };

    double m_dt = 0;
    bool m_hasControlModel = false;
    bool m_continuousControl = false;

    // std::function<Eigen::VectorXd(double timestep, const Eigen::VectorXd&
    // state,
    //                               const Eigen::VectorXd& control,
    //                               const lager::gncpy::control::ControlParams*
    //                               controlParams)>
    //     m_controlModel;
};

// template <typename F>

// template <class Archive>
// inline void INonLinearDynamics::serialize(Archive& ar) {
//     bool tmp = m_hasControlModel;
//     m_hasControlModel = false;
//     ar(cereal::make_nvp("IDynamics",
//                         cereal::virtual_base_class<IDynamics>(this)),
//        CEREAL_NVP(m_dt), CEREAL_NVP(m_hasControlModel),
//        CEREAL_NVP(m_continuousControl));
//     m_hasControlModel = tmp;
// }

}  // namespace lager::gncpy::dynamics
