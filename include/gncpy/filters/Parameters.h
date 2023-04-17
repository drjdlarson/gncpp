#pragma once
#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/polymorphic.hpp>
#include <memory>

#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/Parameters.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::filters {

class BayesPredictParams {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(BayesPredictParams)

   public:
    virtual ~BayesPredictParams() = default;

    std::shared_ptr<lager::gncpy::dynamics::StateTransParams> stateTransParams;
    std::shared_ptr<lager::gncpy::dynamics::ControlParams> controlParams;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(stateTransParams), CEREAL_NVP(controlParams));
    }
};

class BayesCorrectParams {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(BayesCorrectParams)

   public:
    virtual ~BayesCorrectParams() = default;
    std::shared_ptr<lager::gncpy::measurements::MeasParams> measParams;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(measParams));
    }
};

}  // namespace lager::gncpy::filters

CEREAL_REGISTER_TYPE(lager::gncpy::filters::BayesPredictParams)
CEREAL_REGISTER_TYPE(lager::gncpy::filters::BayesCorrectParams)
