#pragma once
#include <boost/serialization/access.hpp>

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
    friend class boost::serialization::access;

    GNCPY_SERIALIZE_CLASS(StateTransParams)

   public:
    virtual ~StateTransParams() = default;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar,
                   [[maybe_unused]] const unsigned int version) {
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
    friend class boost::serialization::access;

    GNCPY_SERIALIZE_CLASS(ConstraintParams)

   public:
    virtual ~ConstraintParams() = default;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar,
                   [[maybe_unused]] const unsigned int version) {
        /* nothing to serialize */
    }
};

}  // namespace lager::gncpy::dynamics
