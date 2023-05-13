#pragma once
#include <Eigen/Dense>
#include <cereal/types/vector.hpp>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/measurements/ILinearMeasModel.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

class StateObservationParams final : public MeasParams {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(StateObservationParams)

   public:
    StateObservationParams() = default;
    explicit StateObservationParams(const std::vector<uint8_t>& obsInds)
        : obsInds(obsInds) {}

    std::vector<uint8_t> obsInds;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("MeasParams",
                            cereal::virtual_base_class<MeasParams>(this)),
           CEREAL_NVP(obsInds));
    }
};

class StateObservation final : public ILinearMeasModel {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(StateObservation)

   public:
    StateObservation() = default;

    Eigen::MatrixXd getMeasMat(
        const Eigen::VectorXd& state,
        const MeasParams* params = nullptr) const override;

   private:
    template <class Archive>
    void serialize(Archive& ar);
};

template <class Archive>
void StateObservation::serialize(Archive& ar) {
    ar(cereal::make_nvp("ILinearMeasModel",
                        cereal::virtual_base_class<ILinearMeasModel>(this)));
}

}  // namespace lager::gncpy::measurements

CEREAL_REGISTER_TYPE(lager::gncpy::measurements::StateObservation)
CEREAL_REGISTER_TYPE(lager::gncpy::measurements::StateObservationParams)
