#include "gncpy/filters/ExtendedKalman.h"

namespace lager::gncpy::filters {
template class ExtendedKalman<float>;
template class ExtendedKalman<double>;

}  // namespace lager::gncpy::filters
