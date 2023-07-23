#pragma once
#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

class IMeasModel {
    friend class boost::serialization::access;

   public:
    virtual ~IMeasModel() = default;
    virtual Eigen::VectorXd measure(
        const Eigen::VectorXd& state,
        const MeasParams* params = nullptr) const = 0;
    virtual Eigen::MatrixXd getMeasMat(
        const Eigen::VectorXd& state,
        const MeasParams* params = nullptr) const = 0;

   private:
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar, [[maybe_unused]] const unsigned int version){
        /* nothing to serializat */
    }
};

}  // namespace lager::gncpy::measurements
