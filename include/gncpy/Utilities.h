#pragma once
#include <memory>

namespace lager::gncpy::utilities {

/**
 * @brief Check if the pointer is an instance of a class type
 *
 * @tparam Base testing class type
 * @tparam T Current class type
 * @param ptr The pointer to a class to check
 * @return true class is an ancestor of type Base
 * @return false
 */
template <typename Base, typename T>
inline bool instanceof (const T *ptr) {
    return dynamic_cast<const Base *>(ptr) != nullptr;
}

template <typename Base, typename T>
inline bool instanceof (const boost::shared_ptr<T> ptr) {
    return boost::dynamic_pointer_cast<const Base>(ptr) != nullptr;
}

}  // namespace lager::gncpy::utilities
