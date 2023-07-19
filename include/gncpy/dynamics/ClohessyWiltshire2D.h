#pragma once
#include <math.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/ILinearDynamics.h"
#include "gncpy/dynamics/Parameters.h"


namespace lager::gncpy::dynamics {

/// @ brief Clohessy Wiltshire relative orbital dynamics model
class ClohessyWiltshire2D : public ILinearDynamics {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(ClohessyWiltshire2D)

    public:
        ClohessyWiltshire2D() = default;
        explicit ClohessyWiltshire2D(double dt, double mean_motion) 
        : m_dt(dt),
         m_mean_motion(mean_motion)
        {

        }

        inline std::vector<std::string> stateNames() const override {
            return std::vector<std::string>{"x pos", "y pos", "x vel", "y vel"};
        };

        Eigen::MatrixXd getStateMat([[maybe_unused]] double timestep,
                                    [[maybe_unused]] const StateTransParams* const 
                                    stateTransParams=nullptr) const override;

        inline double dt() const {return m_dt;}
        inline double mean_motion() const {return m_mean_motion;}
        inline void setDt(double dt) {m_dt = dt;} 
        inline void setMeanMotion(double mean_motion) {m_mean_motion = mean_motion;}
        
        
    protected:
        double m_dt;
        double m_mean_motion;

    private:
        template <class Archive>
        void serialize(Archive& ar);
        // std::shared_ptr<lager::gncpy::control::IControlModel> m_controlModel;

        
};

template <class Archive>
void ClohessyWiltshire2D::serialize(Archive& ar) {
ar(cereal::make_nvp("ILinearDynamics",
                    cereal::virtual_base_class<ILinearDynamics>(this)),
    CEREAL_NVP(m_dt), CEREAL_NVP(m_mean_motion));
}

} // namespace lager::gncpy::dynamics

CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::ClohessyWiltshire2D)