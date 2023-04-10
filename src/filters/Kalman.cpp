#include "gncpy/filters/Kalman.h"

template class lager::gncpy::filters::Kalman<float>;
template class lager::gncpy::filters::Kalman<double>;
CEREAL_REGISTER_DYNAMIC_INIT(gncpy)
GNCPY_SERIALIZE_TYPES(lager::gncpy::filters::Kalman)
