#pragma once
#include <vector>
#include <sstream>

#include <cereal/access.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>

#include "gncpy/math/Matrix.h"
#include "gncpy/dynamics/ILinearDynamics.h"

namespace lager::gncpy::dynamics {

template<typename T>
class DoubleIntegrator final: public ILinearDynamics<T>{

friend class cereal::access;

public:
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

    // see https://stackoverflow.com/questions/42253474/trouble-deserializing-cereal-portablebinaryarchive
    // for details on save/load class state
    std::stringstream saveClassState() {
        std::stringstream ssb(std::ios::in | std::ios::out | std::ios::binary);
        this->createOutputArchive<cereal::PortableBinaryOutputArchive>(ssb);

        return ssb;
    }

    static DoubleIntegrator<T> loadClass(std::stringstream& fState) {
        DoubleIntegrator<T> out;
        createInputArchive<cereal::PortableBinaryInputArchive>(fState, out);
        return std::move(out);
    }

    std::string toJSON() {
        std::stringstream ss(std::ios::out);
        this->createOutputArchive<cereal::JSONOutputArchive>(ss);
        return ss.str();
    }

private:
    DoubleIntegrator<T>() = default;

    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("ILinearDynamics", cereal::virtual_base_class<ILinearDynamics<T>>(this)), CEREAL_NVP(m_dt));
    }

    template<class Archive>
    void createOutputArchive(std::stringstream& os) {
        Archive ar(os);
        ar(*this);
    }

    template<class Archive>
    static void createInputArchive(std::stringstream& is, DoubleIntegrator<T>& cls) {
        Archive ar(is);
        ar(cls);
    }

    T m_dt;

};

} // namespace lager::gncpy::dynamics

CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::DoubleIntegrator<double>);
