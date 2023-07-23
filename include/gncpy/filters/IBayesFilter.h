#pragma once
#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <memory>
#include <optional>

#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/filters/Parameters.h"
#include "gncpy/math/SerializeEigen.h"
#include "gncpy/measurements/IMeasModel.h"

namespace lager::gncpy::filters {

/// @brief Interface for all Bayes filters
class IBayesFilter {
    friend class boost::serialization::access;

   public:
    virtual ~IBayesFilter() = default;

    /**
     * @brief Performs the prediction step of the filter
     *
     * @param timestep current timestep
     * @param curState current state estimate
     * @param controlInput optional control input
     * @param params parameters needed by the prediction step
     * @return Eigen::VectorXd predicted state estimate
     */
    virtual Eigen::VectorXd predict(
        double timestep, const Eigen::VectorXd& curState,
        const std::optional<Eigen::VectorXd> controlInput,
        const BayesPredictParams* params = nullptr) = 0;

    /**
     * @brief Performs the correction step of the filter
     *
     * @param timestep current timestep
     * @param meas measurement vector
     * @param curState current state estimate
     * @param measFitProb output measurement fit probability
     * @param params parameters needed by the correction step
     * @return Eigen::VectorXd corrected state estimate
     */
    virtual Eigen::VectorXd correct(
        double timestep, const Eigen::VectorXd& meas,
        const Eigen::VectorXd& curState, double& measFitProb,
        const BayesCorrectParams* params = nullptr) = 0;

    /**
     * @brief Set the state dynamics model
     *
     * @param dynObj Dynamics object that implements the model
     * @param procNoise Process noise matrix for the filter
     */
    virtual void setStateModel(std::shared_ptr<dynamics::IDynamics> dynObj,
                               Eigen::MatrixXd procNoise) = 0;

    /**
     * @brief Set the measurement model
     *
     * @param measObj Object that implements the measurement model
     * @param measNoise measurement noise matrix for the filter
     */
    virtual void setMeasurementModel(
        std::shared_ptr<measurements::IMeasModel> measObj,
        Eigen::MatrixXd measNoise) = 0;

    virtual std::shared_ptr<dynamics::IDynamics> dynamicsModel() const = 0;
    virtual std::shared_ptr<measurements::IMeasModel> measurementModel()
        const = 0;

    inline virtual Eigen::MatrixXd& getCov() { return m_cov; }
    inline virtual const Eigen::MatrixXd& viewCov() { return m_cov; }

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar& m_cov;
    }

    Eigen::MatrixXd m_cov;
};

}  // namespace lager::gncpy::filters
