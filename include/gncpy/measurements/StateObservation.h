#pragma once
#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/measurements/ILinearMeasModel.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

class StateObservationParams final : public MeasParams {
    friend class boost::serialization::access;

    // GNCPY_SERIALIZE_CLASS(StateObservationParams)

   public:
    StateObservationParams() = default;
    explicit StateObservationParams(const std::vector<uint8_t>& obsInds)
        : obsInds(obsInds) {}

    std::vector<uint8_t> obsInds;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar& boost::serialization::base_object<MeasParams>(*this);
        ar& obsInds;
    }
};

class StateObservation final : public ILinearMeasModel {
    friend class boost::serialization::access;

    // GNCPY_SERIALIZE_CLASS(StateObservation)

   public:
    StateObservation() = default;

    Eigen::MatrixXd getMeasMat(
        const Eigen::VectorXd& state,
        const MeasParams* params = nullptr) const override;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar& boost::serialization::base_object<ILinearMeasModel>(*this);
    };
};

}  // namespace lager::gncpy::measurements
