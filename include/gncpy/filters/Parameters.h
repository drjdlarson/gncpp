#pragma once
#include <boost/serialization/access.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <memory>

#include "gncpy/SerializeMacros.h"
#include "gncpy/control/Parameters.h"
#include "gncpy/dynamics/Parameters.h"
#include "gncpy/measurements/Parameters.h"

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
    friend class boost::serialization::access;

    GNCPY_SERIALIZE_CLASS(BayesPredictParams)

   public:
    virtual ~BayesPredictParams() = default;

    std::shared_ptr<lager::gncpy::dynamics::StateTransParams> stateTransParams;
    std::shared_ptr<lager::gncpy::control::ControlParams> controlParams;

   private:
    template <class Archive>
    void serialize(Archive& ar, [[maybe_unused]] const unsigned int version) {
        ar& stateTransParams;
        ar& controlParams;
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
    friend class boost::serialization::access;

    GNCPY_SERIALIZE_CLASS(BayesCorrectParams)

   public:
    virtual ~BayesCorrectParams() = default;
    std::shared_ptr<lager::gncpy::measurements::MeasParams> measParams;

   private:
    template <class Archive>
    void serialize(Archive& ar, [[maybe_unused]] const unsigned int version) {
        ar& measParams;
    }
};

}  // namespace lager::gncpy::filters
