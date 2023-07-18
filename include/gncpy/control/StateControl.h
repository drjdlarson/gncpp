#pragma once
#include <Eigen/Dense>
#include <cereal/types/vector.hpp>
#include <vector>

#include "gncpy/SerializeMacros.h"
#include "gncpy/control/ILinearControlModel.h"
#include "gncpy/control/Parameters.h"

namespace lager::gncpy::control {

class StateControlParams final : public ControlParams {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(StateControlParams)

    public:
        StateControlParams() = default;
        explicit StateControlParams (const std::vector<uint8_t>& contInds) 
            : contInds(contInds){
                for (const auto& ii : contInds) {
                    vals.push_back(1.0);
                }
            }
        StateControlParams(const std::vector<uint8_t>& contInds, const std::vector<double>& vals)
            : contInds(contInds), vals(vals){}

        std::vector<uint8_t> contInds;
        std::vector<double> vals;

    private:
        template <class Archive>
        void serialize(Archive& ar) {
            ar(cereal::make_nvp("ControlParams",
                                cereal::virtual_base_class<ControlParams>(this)),
                CEREAL_NVP(contInds));
        }
};

class StateControl final : public ILinearControlModel {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(StateControl)

    public:
        StateControl() = default;
        explicit StateControl(size_t stateDim) : m_stateDim(stateDim) {}

    Eigen::MatrixXd getInputMat(double timestep,
        const ControlParams* params=nullptr) const override;

    private:
        size_t m_stateDim;
        template <class Archive>
        void serialize(Archive& ar);
};

template <class Archive>
void StateControl::serialize(Archive& ar) {
    ar(cereal::make_nvp("ILinearControlModel",
                        cereal::virtual_base_class<ILinearControlModel>(this)));
}
}  //  namespace lager::gncpy::control

CEREAL_REGISTER_TYPE(lager::gncpy::control::StateControl)
CEREAL_REGISTER_TYPE(lager::gncpy::control::StateControlParams)