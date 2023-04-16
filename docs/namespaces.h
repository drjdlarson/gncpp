#error "For documentation only"

// see
// https://stackoverflow.com/questions/7385250/doxygen-not-listing-nested-namespaces
// and
// https://stackoverflow.com/questions/3525631/documenting-namespaces-that-span-multiple-files-doxygen

/**
 * @namespace lager
 * @brief Top level namespace for all LAGER code
 *
 */
namespace lager {

/**
 * @namespace lager::gncpy
 * @brief Contains code for GNC routines
 *
 * This is the main namespace for the library and contains all the needed code
 * for GNC routines.
 *
 */
namespace gncpy {

/**
 * @namespace lager::gncpy::dynamics
 * @brief Contains dynamics models
 *
 */
namespace dynamics {}

/**
 * @namespace lager::gncpy::filters
 * @brief Contains the filter implementations
 *
 */
namespace filters {}

/**
 * @namespace lager::gncpy::matrix
 * @brief Contains matrix and vector related code
 *
 */
namespace matrix {}

/**
 * @namespace lager::gncpy::exceptions
 * @brief Contains general exceptions applicable to all GNCPy
 *
 */
namespace exceptions {}

/**
 * @namespace lager::gncpy::math
 * @brief Contains math routines
 *
 */
namespace math {}

/**
 * @namespace lager::gncpy::measurements
 * @brief Contains measurement models
 *
 */
namespace measurements {}

/**
 * @namespace lager::gncpy::utility
 * @brief Contains utility functions for all GNCPy
 *
 */
namespace measurements {}

}  // namespace gncpy

}  // namespace lager
