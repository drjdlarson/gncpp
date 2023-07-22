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
    explicit StateControlParams(const std::vector<uint8_t>& contRows,
                                const std::vector<uint8_t>& contColumns)
        : contRows(contRows), contColumns(contColumns) {
        for (const auto& ii : contRows) {
            vals.push_back(1.0);
        }
    }
    StateControlParams(const std::vector<uint8_t>& contRows,
                       const std::vector<uint8_t>& contColumns,
                       const std::vector<double>& vals)
        : contRows(contRows), contColumns(contColumns), vals(vals) {}

    std::vector<uint8_t> contRows;
    std::vector<uint8_t> contColumns;
    std::vector<double> vals;

   private:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("ControlParams",
                            cereal::virtual_base_class<ControlParams>(this)),
           CEREAL_NVP(contRows), CEREAL_NVP(contColumns), CEREAL_NVP(vals));
    }
};

class StateControl final : public ILinearControlModel {
    friend class cereal::access;

    GNCPY_SERIALIZE_CLASS(StateControl)

   public:
    StateControl() = default;
    explicit StateControl(size_t stateDim, size_t contDim)
        : m_stateDim(stateDim), m_contDim(contDim) {}

    Eigen::MatrixXd getInputMat(
        double timestep, const ControlParams* params = nullptr) const override;

   private:
    size_t m_stateDim;
    size_t m_contDim;
    template <class Archive>
    void serialize(Archive& ar);
};

template <class Archive>
void StateControl::serialize(Archive& ar) {
    ar(cereal::make_nvp("ILinearControlModel",
                        cereal::virtual_base_class<ILinearControlModel>(this)),
       CEREAL_NVP(m_stateDim), CEREAL_NVP(m_contDim));
}
}  //  namespace lager::gncpy::control

CEREAL_REGISTER_TYPE(lager::gncpy::control::StateControl)
CEREAL_REGISTER_TYPE(lager::gncpy::control::StateControlParams)