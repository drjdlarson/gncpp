#pragma once
#include <stdexcept>

namespace lager::gncpy::exceptions {

/// @brief General exception for when incorrect parameters are supplied.
class BadParams final : public std::runtime_error {
   public:
    explicit BadParams(char const* const message) noexcept;
};

/// @brief General exception for when an incorrect type is used
class TypeError final : public std::runtime_error {
   public:
    explicit TypeError(char const* const message) noexcept;
};

}  // namespace lager::gncpy::exceptions