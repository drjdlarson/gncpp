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

/**
 * @brief Implements a Curvilinear motion model
 *
 * This is a slight variation from normal INonLinearDynamics classes
 * because it hardcodes a control model. Also, the angle state should be kept
 * between 0-360 degrees. This implements the following system of ODEs
 *
 * \f{align}{
 *      \dot{x} &= v cos(\psi) \\
 *      \dot{y} &= v sin(\psi) \\
 *      \dot{v} &= u_0 \\
 *      \dot{\psi} &= u_1
 * \f}
 *
 * See \cite Li2000_SurveyofManeuveringTargetTrackingDynamicModels for details.
 *
 */
template <typename T>
class CurvilinearMotion final : public INonLinearDynamics<T> {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(CurvilinearMotion<T>)

   public:
    CurvilinearMotion();

    std::vector<std::string> stateNames() const override {
        return std::vector<std::string>{"x pos", "y pos", "speed",
                                        "turn angle"};
    }

    matrix::Vector<T> continuousDynamics(
        [[maybe_unused]] T timestep, const matrix::Vector<T>& state,
        [[maybe_unused]] const StateTransParams* stateTransParams =
            nullptr) const override;
    template <typename F>
    void setControlModel([[maybe_unused]] F&& model,
                         [[maybe_unused]] bool continuousModel);
    void clearControlModel() override;

   private:
    template <class Archive>
    void serialize(Archive& ar);
};

template <typename T>
CurvilinearMotion<T>::CurvilinearMotion() {
    INonLinearDynamics<T>::setControlModel(
        []([[maybe_unused]] T timestep,
           [[maybe_unused]] const matrix::Vector<T>& state,
           const matrix::Vector<T>& control,
           [[maybe_unused]] const ControlParams* controlParams = nullptr) {
            return matrix::Vector<T>(
                {static_cast<T>(0), static_cast<T>(0), control(0), control(1)});
        },
        true);
}

template <typename T>
matrix::Vector<T> CurvilinearMotion<T>::continuousDynamics(
    [[maybe_unused]] T timestep, const matrix::Vector<T>& state,
    [[maybe_unused]] const StateTransParams* stateTransParams) const {
    return matrix::Vector<T>({state(2) * static_cast<T>(cos(state(3))),
                              state(2) * static_cast<T>(sin(state(3))),
                              static_cast<T>(0), static_cast<T>(0)});
}

template <typename T>
template <typename F>
void CurvilinearMotion<T>::setControlModel(
    [[maybe_unused]] F&& model, [[maybe_unused]] bool continuousModel) {
    std::cerr << "Can not set control model for curvilinear motion "
                 "dynamics. It has a fixed control model!"
              << std::endl;
}

template <typename T>
void CurvilinearMotion<T>::clearControlModel() {
    std::cerr << "Warning, disabling control model and it can not be "
                 "re-enabled for this curvilinear motion model!"
              << std::endl;
    INonLinearDynamics<T>::clearControlModel();
}

template <typename T>
template <class Archive>
void CurvilinearMotion<T>::serialize(Archive& ar) {
    ar(cereal::make_nvp(
        "INonLinearDynamics",
        cereal::virtual_base_class<INonLinearDynamics<T>>(this)));
}

extern template class CurvilinearMotion<float>;
extern template class CurvilinearMotion<double>;

}  // namespace lager::gncpy::dynamics

GNCPY_REGISTER_SERIALIZE_TYPES(lager::gncpy::dynamics::CurvilinearMotion)
