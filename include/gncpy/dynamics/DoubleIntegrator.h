#pragma once
#include <sstream>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/ILinearDynamics.h"
#include "gncpy/math/Matrix.h"

namespace lager::gncpy::dynamics {

/**
 * @brief Double integrator dynamics
 *
 * @tparam T fundamental data type for the calculations
 */
template <typename T>
class DoubleIntegrator final : public ILinearDynamics<T> {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(DoubleIntegrator<T>)

   public:
    DoubleIntegrator<T>() = default;

    inline std::vector<std::string> stateNames() const {
        return std::vector<std::string>{"x pos", "y pos", "x vel", "y vel"};
    };

    explicit DoubleIntegrator(T dt) : m_dt(dt) {}

    matrix::Matrix<T> getStateMat(
        [[maybe_unused]] T timestep,
        [[maybe_unused]] const StateTransParams* const stateTransParams =
            nullptr) const override {
        matrix::Matrix<T> F(
            {{1, 0, m_dt, 0}, {0, 1, 0, m_dt}, {0, 0, 1, 0}, {0, 0, 0, 1}});

        return F;
    }

    inline T dt() const { return m_dt; }
    inline void setDt(T dt) { m_dt = dt; }

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp(
               "ILinearDynamics",
               cereal::virtual_base_class<ILinearDynamics<T>>(this)),
           CEREAL_NVP(m_dt));
    }

    T m_dt;
};

}  // namespace lager::gncpy::dynamics

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::dynamics::DoubleIntegrator)
