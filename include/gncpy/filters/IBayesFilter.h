#pragma once
#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>
#include <concepts>
#include <memory>
#include <optional>

#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/filters/Parameters.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"
#include "gncpy/measurements/IMeasModel.h"

namespace lager::gncpy::filters {

/// @brief Interface for all Bayes filters
template <typename T>
    requires std::integral<T> || std::floating_point<T>
class IBayesFilter {
    friend class cereal::access;

   public:
    virtual ~IBayesFilter() = default;

    /**
     * @brief Performs the prediction step of the filter
     *
     * @param timestep current timestep
     * @param curState current state estimate
     * @param controlInput optional control input
     * @param params parameters needed by the prediction step
     * @return matrix::Vector<T> predicted state estimate
     */
    virtual matrix::Vector<T> predict(
        T timestep, const matrix::Vector<T>& curState,
        const std::optional<matrix::Vector<T>> controlInput,
        const BayesPredictParams* params = nullptr) = 0;

    /**
     * @brief Performs the correction step of the filter
     *
     * @param timestep current timestep
     * @param meas measurement vector
     * @param curState current state estimate
     * @param measFitProb output measurement fit probability
     * @param params parameters needed by the correction step
     * @return matrix::Vector<T> corrected state estimate
     */
    virtual matrix::Vector<T> correct(
        T timestep, const matrix::Vector<T>& meas,
        const matrix::Vector<T>& curState, T& measFitProb,
        const BayesCorrectParams* params = nullptr) = 0;

    /**
     * @brief Set the state dynamics model
     *
     * @param dynObj Dynamics object that implements the model
     * @param procNoise Process noise matrix for the filter
     */
    virtual void setStateModel(std::shared_ptr<dynamics::IDynamics<T>> dynObj,
                               matrix::Matrix<T> procNoise) = 0;

    /**
     * @brief Set the measurement model
     *
     * @param measObj Object that implements the measurement model
     * @param measNoise measurement noise matrix for the filter
     */
    virtual void setMeasurementModel(
        std::shared_ptr<measurements::IMeasModel<T>> measObj,
        matrix::Matrix<T> measNoise) = 0;

    virtual std::shared_ptr<dynamics::IDynamics<T>> dynamicsModel() const = 0;
    virtual std::shared_ptr<measurements::IMeasModel<T>> measurementModel()
        const = 0;

    matrix::Matrix<T> cov;

   private:
    template <class Archive>
    void serialize(Archive& ar);
};

template <typename T>
    requires std::integral<T> || std::floating_point<T>
template <class Archive>
void IBayesFilter<T>::serialize(Archive& ar) {
    ar(CEREAL_NVP(cov));
}

}  // namespace lager::gncpy::filters

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::filters::IBayesFilter)
