#include "gncpy/filters/Kalman.h"

namespace lager::gncpy::filters {
template class Kalman<float>;
template class Kalman<double>;

}  // namespace lager::gncpy::filters
