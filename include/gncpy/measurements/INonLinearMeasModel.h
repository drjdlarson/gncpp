#pragma once
#include <Eigen/Dense>
#include <functional>
#include <vector>

// #include "gncpy/SerializeMacros.h"
#include "gncpy/measurements/IMeasModel.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

class INonLinearMeasModel : public IMeasModel {
    // friend class cereal::access;

   public:
    Eigen::VectorXd measure(const Eigen::VectorXd& state,
                            const MeasParams* params = nullptr) const override;
    Eigen::MatrixXd getMeasMat(
        const Eigen::VectorXd& state,
        const MeasParams* params = nullptr) const override;

   protected:
    virtual std::vector<std::function<double(const Eigen::VectorXd&)>>
    getMeasFuncLst(const MeasParams* params = nullptr) const = 0;

   private:
    // template <class Archive>
    // void serialize([[maybe_unused]] Archive& ar);
};

// template <class Archive>
// void INonLinearMeasModel::serialize([[maybe_unused]] Archive& ar) {
//     ar(cereal::make_nvp("IMeasModel",
//                         cereal::virtual_base_class<IMeasModel>(this)));
// }

}  // namespace lager::gncpy::measurements

// CEREAL_REGISTER_TYPE(lager::gncpy::measurements::INonLinearMeasModel)
