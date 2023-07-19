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
#include "gncpy/control/Parameters.h"

namespace lager::gncpy::filters {

/**
 * @brief Base polymorphic class for the prediction step of bayes filters.
 *
 * This can be subclassed if needed to allow for other parameters for specific
 * filter implementations. Most of these modifications should be able to be
 * taken care of by the polymorphic state transition and control parameter
 * object members.
 *
 */
class BayesPredictParams {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(BayesPredictParams)

   public:
    virtual ~BayesPredictParams() = default;

    std::shared_ptr<lager::gncpy::dynamics::StateTransParams> stateTransParams;
    std::shared_ptr<lager::gncpy::control::ControlParams> controlParams;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(stateTransParams), CEREAL_NVP(controlParams));
    }
};

/**
 * @brief Base polymorphic class for the correction step of bayes filters.
 *
 * This can be subclassed if needed to allow for other parameters for specific
 * filter implementations. Most of these modifications should be able to be
 * taken care of by the polymorphic measurement parameter object members.
 *
 */
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
