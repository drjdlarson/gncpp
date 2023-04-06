#pragma once
#include <vector>
#include <sstream>

#include <cereal/archives/portable_binary.hpp>

#include "gncpy/math/Matrix.h"
#include "gncpy/dynamics/ILinearDynamics.h"

namespace lager::gncpy::dynamics {

template<typename T>
class DoubleIntegrator final: public ILinearDynamics<T>{
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

    template <class Archive>
    void serialize(Archive& ar) {
        ar(m_dt);
        // ar(cereal::base_class<ILinearDynamics<T>>(this), m_dt);
    }

    // see https://stackoverflow.com/questions/22799551/sending-a-stringstream-of-binary-data-from-cereal-with-enet
    // for details on save/load filter state
    char const* saveFilterState() {
        std::ostringstream os;
        this->createOutputArchive(os);

        return os.str().c_str();
    }

    static DoubleIntegrator<T> loadFilterState(char const* fState) {
        DoubleIntegrator<T> out;
        std::istringstream is(fState);
        createInputArchive(is, out);
        return std::move(out);
    }

private:
    DoubleIntegrator() = default;

    void createOutputArchive(std::ostringstream& os) {
        cereal::PortableBinaryOutputArchive ar(os);
        ar(*this);
    }

    static void createInputArchive(std::istringstream& is, DoubleIntegrator<T>& cls) {
        cereal::PortableBinaryInputArchive ar(is);
        ar(cls);
    }

    T m_dt;
};

} // namespace lager::gncpy::dynamics 