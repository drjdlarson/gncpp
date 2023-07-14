#pragma once
#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>

#include "gncpy/SerializeMacros.h"

namespace lager::gncpy::dynamics {

/**
 * @brief Base class for all state transition parameters
 *
 * Polymorphic base class such that any child class can be passed into filters,
 * dynamics, etc. and the object using the child can dynamic cast to the
 * appropriate polymorphic type.
 *
 */
class StateTransParams {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(StateTransParams)

   public:
    virtual ~StateTransParams() = default;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar) {
        /* nothing to serialize */
    }
};

/**
 * @brief Base class for all state constraint parameters
 *
 * Polymorphic base class such that any child class can be passed into filters,
 * dynamics, etc. and the object using the child can dynamic cast to the
 * appropriate polymorphic type.
 *
 */
class ConstraintParams {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(ConstraintParams)

   public:
    virtual ~ConstraintParams() = default;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar) {
        /* nothing to serialize */
    }
};

}  // namespace lager::gncpy::dynamics

CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::StateTransParams)
CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::ControlParams)
CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::ConstraintParams)
