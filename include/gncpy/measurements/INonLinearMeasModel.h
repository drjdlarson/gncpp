#pragma once
#include <Eigen/Dense>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <functional>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/measurements/IMeasModel.h"
#include "gncpy/measurements/Parameters.h"

namespace lager::gncpy::measurements {

class INonLinearMeasModel : public IMeasModel {
    friend class boost::serialization::access;

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
    template <class Archive>
    void serialize([[maybe_unused]] Archive& ar,
                   [[maybe_unused]] const unsigned int version) {
        ar& boost::serialization::base_object<IMeasModel>(*this);
    }
};

}  // namespace lager::gncpy::measurements
