#pragma once
#include <vector>
#include <sstream>

#include "gncpy/dynamics/ILinearDynamics.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/SerializeMacros.h"


namespace lager::gncpy::dynamics {

template<typename T>
class DoubleIntegrator final: public ILinearDynamics<T>{

friend class cereal::access;

GNCPY_SERIALIZE_CLASS(DoubleIntegrator<T>)

public:
    DoubleIntegrator<T>() = default;

    inline std::vector<std::string> stateNames() const { return std::vector<std::string>{"x pos", "y pos", "x vel", "y vel"}; };

    explicit DoubleIntegrator(T dt)
    : m_dt(dt) {

    }

    matrix::Matrix<T> getStateMat([[maybe_unused]] T timestep, [[maybe_unused]] const StateTransParams* const stateTransParams=nullptr) const override{
        matrix::Matrix<T> F({{1, 0, m_dt, 0},
                             {0, 1, 0, m_dt},
                             {0, 0, 1, 0},
                             {0, 0, 0, 1}});

        return F;
    }

    inline T dt() const { return m_dt; }
    inline void setDt(T dt) { m_dt = dt; }

private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("ILinearDynamics", cereal::virtual_base_class<ILinearDynamics<T>>(this)), CEREAL_NVP(m_dt));
    }

    T m_dt;

};

} // namespace lager::gncpy::dynamics

// see https://uscilab.github.io/cereal/assets/doxygen/polymorphic_8hpp.html#a01ebe0f840ac20c307f64622384e4dae
// and "Registering from a source file" here https://uscilab.github.io/cereal/polymorphism.html
// CEREAL_FORCE_DYNAMIC_INIT(DoubleIntegrator)
GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::dynamics::DoubleIntegrator)
