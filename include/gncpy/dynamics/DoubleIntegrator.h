#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/ILinearDynamics.h"
#include "gncpy/dynamics/Parameters.h"

namespace lager::gncpy::dynamics {

/// @brief Double integrator dynamics
class DoubleIntegrator final : public ILinearDynamics {
    friend class boost::serialization::access;

    // GNCPY_SERIALIZE_CLASS(DoubleIntegrator)

   public:
    DoubleIntegrator() = default;
    explicit DoubleIntegrator(double dt) : m_dt(dt) {}

    inline std::vector<std::string> stateNames() const override {
        return std::vector<std::string>{"x pos", "y pos", "x vel", "y vel"};
    };

    Eigen::MatrixXd getStateMat([[maybe_unused]] double timestep,
                                [[maybe_unused]] const StateTransParams* const
                                    stateTransParams = nullptr) const override;

    inline double dt() const { return m_dt; }
    inline void setDt(double dt) { m_dt = dt; }

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar& boost::serialization::base_object<ILinearDynamics>(*this);
        ar& m_dt;
    }

    double m_dt;
    // std::shared_ptr<lager::gncpy::control::IControlModel> m_controlModel;
};

}  // namespace lager::gncpy::dynamics
