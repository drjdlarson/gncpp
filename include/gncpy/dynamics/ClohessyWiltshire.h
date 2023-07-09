#pragma once
#include <math.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/ILinearDynamics.h"
#include "gncpy/dynamics/ClohessyWiltshire2D.h"
#include "gncpy/dynamics/Parameters.h"


namespace lager::gncpy::dynamics {

/// @ brief Clohessy Wiltshire relative orbital dynamics model
class ClohessyWiltshire : public ClohessyWiltshire2D {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(ClohessyWiltshire)

    public:
        ClohessyWiltshire() = default;
        using ClohessyWiltshire2D::ClohessyWiltshire2D;

        inline std::vector<std::string> stateNames() const override {
            return std::vector<std::string>{"x pos", "y pos", "z pos", "x vel", "y vel", "z vel"};
        };

        Eigen::MatrixXd getStateMat([[maybe_unused]] double timestep,
                                    [[maybe_unused]] const StateTransParams* const 
                                    stateTransParams=nullptr) const override;

    private:
        template <class Archive>
        void serialize(Archive& ar);
};

template <class Archive>
void ClohessyWiltshire::serialize(Archive& ar) {
    ar(cereal::make_nvp("ClohessyWiltshire2D",
                        cereal::virtual_base_class<ClohessyWiltshire2D>(this)));
    }

} // namespace lager::gncpy::dynamics

CEREAL_REGISTER_TYPE(lager::gncpy::dynamics::ClohessyWiltshire)