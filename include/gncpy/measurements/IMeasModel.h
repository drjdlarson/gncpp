#pragma once
#include <Eigen/Dense>
#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>
#include <concepts>

#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

class IMeasModel {
    friend class cereal::access;

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
    void serialize(Archive& ar);
};

template <class Archive>
void IMeasModel::serialize([[maybe_unused]] Archive& ar) {
    /* nothing to save*/
}

}  // namespace lager::gncpy::measurements

CEREAL_REGISTER_TYPE(lager::gncpy::measurements::IMeasModel)
