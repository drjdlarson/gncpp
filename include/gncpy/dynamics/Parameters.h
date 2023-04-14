#pragma once
#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>

#include "gncpy/SerializeMacros.h"

namespace lager::gncpy::dynamics {

class StateTransParams {
  friend class cereal::access;

  GNCPY_SERIALIZE_CLASS(StateTransParams)

 public:
  virtual ~StateTransParams() = default;

 private:
  template <class Archive>
  void serialize([[maybe_unused]] Archive& ar) {}
};

class ControlParams {
  friend class cereal::access;

  GNCPY_SERIALIZE_CLASS(ControlParams)

 public:
  virtual ~ControlParams() = default;

 private:
  template <class Archive>
  void serialize([[maybe_unused]] Archive& ar) {}
};

class ConstraintParams {
  friend class cereal::access;

  GNCPY_SERIALIZE_CLASS(ConstraintParams)

 public:
  virtual ~ConstraintParams() = default;

 private:
  template <class Archive>
  void serialize([[maybe_unused]] Archive& ar) {}
};

}  // namespace lager::gncpy::dynamics

CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::StateTransParams)
CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::ControlParams)
CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::ConstraintParams)
