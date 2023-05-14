#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/ILinearDynamics.h"
#include "gncpy/dynamics/Parameters.h"

namespace lager::gncpy::dynamics {

/// @brief Double integrator dynamics
class DoubleIntegrator final : public ILinearDynamics {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(DoubleIntegrator)

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
    void serialize(Archive& ar);

    double m_dt;
};

template <class Archive>
void DoubleIntegrator::serialize(Archive& ar) {
    ar(cereal::make_nvp("ILinearDynamics",
                        cereal::virtual_base_class<ILinearDynamics>(this)),
       CEREAL_NVP(m_dt));
}

}  // namespace lager::gncpy::dynamics

CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::DoubleIntegrator)
