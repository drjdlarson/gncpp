#pragma once
#include <Eigen/Dense>
// #include <cereal/access.hpp>

// #include "gncpy/SerializeMacros.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>

#include "gncpy/dynamics/IDynamics.h"
#include "gncpy/filters/Kalman.h"
#include "gncpy/filters/Parameters.h"
#include "gncpy/measurements/ILinearMeasModel.h"
#include "gncpy/measurements/IMeasModel.h"
#include "gncpy/measurements/INonLinearMeasModel.h"

namespace lager::gncpy::filters {

class ExtendedKalman final : public Kalman {
    friend class boost::serialization::access;

    // GNCPY_SERIALIZE_CLASS(ExtendedKalman)

   public:
    Eigen::VectorXd predict(
        double timestep, const Eigen::VectorXd& curState,
        const std::optional<Eigen::VectorXd> controlInput,
        const BayesPredictParams* params = nullptr) override;

    void setStateModel(std::shared_ptr<dynamics::IDynamics> dynObj,
                       Eigen::MatrixXd procNoise) override;

    void setMeasurementModel(std::shared_ptr<measurements::IMeasModel> measObj,
                             Eigen::MatrixXd measNoise) override;

    std::shared_ptr<dynamics::IDynamics> dynamicsModel() const override;
    std::shared_ptr<measurements::IMeasModel> measurementModel() const override;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar& boost::serialization::base_object<Kalman>(*this);
        ar& m_dynOb;
        ar& m_measObj;
        m_continuousCov;
    }

    bool m_continuousCov = false;

    Eigen::MatrixXd m_measNoise;
    std::shared_ptr<dynamics::IDynamics> m_dynObj;
    std::shared_ptr<measurements::IMeasModel> m_measObj;
};

}  // namespace lager::gncpy::filters
