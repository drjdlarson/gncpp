#pragma once
#include <Eigen/Dense>
#include <functional>

#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/dynamics/Parameters.h"

namespace lager::gncpy::dynamics {

/// @brief Interface for all non-linear dynamics models
class INonLinearDynamics : public IDynamics {
    friend class cereal::access;

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
        const Eigen::VectorXd& control) const override;
    Eigen::VectorXd propagateState(
        double timestep, const Eigen::VectorXd& state,
        const Eigen::VectorXd& control,
        const StateTransParams* const stateTransParams,
        const ControlParams* const controlParams,
        const ConstraintParams* const constraintParams) const final;

    template <typename F>
    void setControlModel(F&& model, bool continuousModel);
    void clearControlModel() override { m_hasControlModel = false; }
    bool hasControlModel() const override { return m_hasControlModel; }

    Eigen::MatrixXd getStateMat(
        double timestep, const Eigen::VectorXd& state,
        const StateTransParams* stateTransParams = nullptr) const;

    double dt() const { return m_dt; }
    void setDt(double dt) { m_dt = dt; }

   private:
    // NOTE: can not serialize std::function or lambda function
    // see
    // https://stackoverflow.com/questions/57095837/serialize-lambda-functions-with-cereal
    template <class Archive>
    void serialize(Archive& ar);

    double m_dt = 0;
    bool m_hasControlModel = false;
    bool m_continuousControl = false;
    std::function<Eigen::VectorXd(double timestep, const Eigen::VectorXd& state,
                                  const Eigen::VectorXd& control,
                                  const ControlParams* controlParams)>
        m_controlModel;
};

template <typename F>
void INonLinearDynamics::setControlModel(F&& model, bool continuousModel) {
    m_hasControlModel = true;
    m_continuousControl = continuousModel;
    m_controlModel = std::forward<F>(model);
}

template <class Archive>
inline void INonLinearDynamics::serialize(Archive& ar) {
    bool tmp = m_hasControlModel;
    m_hasControlModel = false;
    ar(cereal::make_nvp("IDynamics",
                        cereal::virtual_base_class<IDynamics>(this)),
       CEREAL_NVP(m_dt), CEREAL_NVP(m_hasControlModel),
       CEREAL_NVP(m_continuousControl));
    m_hasControlModel = tmp;
}

}  // namespace lager::gncpy::dynamics

CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::INonLinearDynamics)
