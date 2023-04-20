#pragma once
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/dynamics/INonLinearDynamics.h"
#include "gncpy/dynamics/Parameters.h"
#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"

namespace lager::gncpy::dynamics {
template <typename T>
class CurvilinearMotion final : public INonLinearDynamics<T> {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(CurvilinearMotion<T>)

   public:
    CurvilinearMotion() {
        INonLinearDynamics<T>::setControlModel(
            []([[maybe_unused]] T timestep,
               [[maybe_unused]] const matrix::Vector<T>& state,
               const matrix::Vector<T>& control,
               [[maybe_unused]] const ControlParams* controlParams = nullptr) {
                return matrix::Vector<T>({static_cast<T>(0), static_cast<T>(0),
                                          control(0), control(1)});
            },
            true);
    }

    std::vector<std::string> stateNames() const override {
        return std::vector<std::string>{"x pos", "y pos", "speed",
                                        "turn angle"};
    }

    matrix::Vector<T> continuousDynamics(
        T timestep, const matrix::Vector<T>& state,
        const StateTransParams* stateTransParams = nullptr) const override {
        return matrix::Vector<T>({state(2) * static_cast<T>(cos(state(3))),
                                  state(2) * static_cast<T>(sin(state(3))),
                                  static_cast<T>(0), static_cast<T>(0)});
    }

    template <typename F>
    void setControlModel([[maybe_unused]] F&& model,
                         [[maybe_unused]] bool continuousModel) {
        std::cerr << "Can not set control model for curvilinear motion "
                     "dynamics. It has a fixed control model!"
                  << std::endl;
    }

    void clearControlModel() override {
        std::cerr << "Warning, disabling control model and it can not be "
                     "re-enabled for this curvilinear motion model!"
                  << std::endl;
        INonLinearDynamics<T>::clearControlModel();
    }

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp(
            "INonLinearDynamics",
            cereal::virtual_base_class<INonLinearDynamics<T>>(this)));
    }
};
}  // namespace lager::gncpy::dynamics

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::dynamics::CurvilinearMotion)
