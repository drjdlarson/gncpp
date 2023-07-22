#pragma once
#include <Eigen/Dense>
// #include <cereal/types/memory.hpp>
#include <memory>
#include <optional>

// #include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/dynamics/ILinearDynamics.h"
#include "gncpy/filters/IBayesFilter.h"
#include "gncpy/filters/Parameters.h"
#include "gncpy/math/Math.h"
// #include "gncpy/math/SerializeEigen.h"
#include "gncpy/measurements/ILinearMeasModel.h"
#include "gncpy/measurements/IMeasModel.h"

namespace lager::gncpy::filters {

/**
 * @brief Implements a Kalman Filter
 *
 * This is a discrete time KF loosely based on
 * @cite Crassidis2011_OptimalEstimationofDynamicSystems
 *
 */
class Kalman : public IBayesFilter {
    // friend class cereal::access;

    // GNCPY_SERIALIZE_CLASS(Kalman)

   public:
    /**
     * @brief Implements a discrete time prediction step
     *
     * @param timestep current timestep
     * @param curState current state estmiate
     * @param controlInput optional control input
     * @param params prediction parameters
     * @return Eigen::VectorXd predicted state estimate
     */
    Eigen::VectorXd predict(
        double timestep, const Eigen::VectorXd& curState,
        [[maybe_unused]] const std::optional<Eigen::VectorXd> controlInput,
        const BayesPredictParams* params = nullptr) override;

    /**
     * @brief Implements a discrete time correction step
     *
     * @param timestep current timestep
     * @param meas current measurement
     * @param curState current state estimate
     * @param measFitProb ouptut measurement fit probability
     * @param params correction step parameters
     * @return Eigen::VectorXd corrected state estimate
     */
    Eigen::VectorXd correct(
        [[maybe_unused]] double timestep, const Eigen::VectorXd& meas,
        const Eigen::VectorXd& curState, double& measFitProb,
        const BayesCorrectParams* params = nullptr) override;

    /**
     * @brief Sets the state model equation for the filter.
     *
     * The dynamics must be linear and may be constant or time varying. This
     * assumes a discrete model of the form
     *
     * \f[
     *      x_{k+1} = F(t) x_k + G(t) u_k
     * \f]
     *
     * @param dynObj Linear dynamics of type dynamics::ILinearDynamics
     * @param procNoise Process noise matrix for the filter
     */
    void setStateModel(std::shared_ptr<dynamics::IDynamics> dynObj,
                       Eigen::MatrixXd procNoise) override;

    /**
     * @brief Sets the measurement model for the filter.
     *
     * This can either set the constant measurement matrix, or the matrix can be
     * time varying. This assumes a measurement model of the form
     *
     * \f[
     *      \tilde{y}_{k+1} = H(t) x_{k+1}^-
     * \f]
     *
     * @param measObj linear measurement model of type
     * measurements::ILinearMeasModel
     * @param measNoise measurement nosie matrix for the filter
     */
    void setMeasurementModel(std::shared_ptr<measurements::IMeasModel> measObj,
                             Eigen::MatrixXd measNoise) override;

    std::shared_ptr<dynamics::IDynamics> dynamicsModel() const override;
    std::shared_ptr<measurements::IMeasModel> measurementModel() const override;

   protected:
    Eigen::MatrixXd m_procNoise;

   private:
    // template <class Archive>
    // void serialize(Archive& ar);

    Eigen::MatrixXd m_measNoise;

    std::shared_ptr<dynamics::ILinearDynamics> m_dynObj;
    std::shared_ptr<measurements::ILinearMeasModel> m_measObj;
};

// template <class Archive>
// void Kalman::serialize(Archive& ar) {
//     ar(cereal::make_nvp("IBayesFilter",
//                         cereal::virtual_base_class<IBayesFilter>(this)),
//        CEREAL_NVP(m_measNoise), CEREAL_NVP(m_procNoise), CEREAL_NVP(m_dynObj),
//        CEREAL_NVP(m_measObj));
// }

}  // namespace lager::gncpy::filters

// CEREAL_REGISTER_TYPE(lager::gncpy::filters::Kalman)
