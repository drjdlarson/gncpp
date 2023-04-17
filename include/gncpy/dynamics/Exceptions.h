#pragma once
#include <stdexcept>

namespace lager::gncpy::dynamics {

/// @brief No control model is given
class NoControlError final : public std::runtime_error {
   public:
    NoControlError() noexcept;
};

/// @brief No stat constraint is given
class NoStateConstraintError final : public std::runtime_error {
   public:
    NoStateConstraintError() noexcept;
};

}  // namespace lager::gncpy::dynamics
