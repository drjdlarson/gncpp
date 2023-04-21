#pragma once
#include <stdexcept>

namespace lager::gncpy::matrix {

/// @brief Runtime error for indexing out of range
class BadIndex final : public std::runtime_error {
   public:
    explicit BadIndex(char const* const message) noexcept;
};

/// @brief Runtime error for inmproper dimensions
class BadDimension final : public std::runtime_error {
   public:
    BadDimension() noexcept;
    explicit BadDimension(char const* const message) noexcept;
};

}  // namespace lager::gncpy::matrix
