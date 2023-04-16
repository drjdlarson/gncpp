#pragma once

// The gncpy library version in the form major * 10000 + minor * 100 + patch.
#define GNCPY_VERSION 000102

#ifndef GNCPY_BEGIN_NAMESPACE
#define GNCPY_BEGIN_NAMESPACE \
  namespace lager::gncpy {    \
  inline namespace v0 {
#define GNCPY_END_NAMESPACE \
  }                         \
  }
#endif