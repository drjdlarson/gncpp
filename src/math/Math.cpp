#include "gncpy/math/Math.h"

namespace lager::gncpy::math {

Eigen::VectorXd getGradient(
    const Eigen::VectorXd& x,
    std::function<double(const Eigen::VectorXd&)> const& fnc) {
    const double step = 1e-7;
    const double invStep2 = 1. / (2. * step);

    Eigen::VectorXd out(x.size());
    Eigen::VectorXd xR(x);
    Eigen::VectorXd xL(x);
    for (uint8_t ii = 0; ii < x.size(); ii++) {
        xR(ii) += step;
        xL(ii) -= step;

        out(ii) = (fnc(xR) - fnc(xL)) * invStep2;

        // reset for next round
        xR(ii) -= step;
        xL(ii) += step;
    }

    return out;
}

Eigen::MatrixXd getJacobian(
    const Eigen::VectorXd& x,
    const std::vector<std::function<double(const Eigen::VectorXd&)>>& fncLst) {
    Eigen::MatrixXd out(fncLst.size(), x.size());
    size_t row = 0;
    size_t col = 0;
    for (auto const& f : fncLst) {
        for (auto const& val : getGradient(x, f)) {
            out(row, col) = val;
            col++;
        }
        row++;
        col = 0;
    }

    return out;
}

Eigen::MatrixXd getJacobian(
    const Eigen::VectorXd& x,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> const& fnc,
    size_t numFunOutputs) {
    Eigen::MatrixXd out(numFunOutputs, x.size());
    size_t col = 0;

    for (size_t row = 0; row < numFunOutputs; row++) {
        auto fi = [&fnc, row](const Eigen::VectorXd& x_) {
            return fnc(x_)(row);
        };
        for (auto& val : getGradient(x, fi)) {
            out(row, col) = val;
            col++;
        }
        col = 0;
    }

    return out;
}

double calcGaussianPDF(const Eigen::VectorXd& x, const Eigen::VectorXd& m,
                       const Eigen::MatrixXd& cov) {
    size_t nDim = x.size();
    double val;
    if (nDim > 1) {
        Eigen::VectorXd diff = x - m;
        val = -0.5 * (static_cast<double>(nDim) * std::log(2.0 * M_PI) +
                      std::log(cov.determinant()) +
                      (diff.transpose() * cov.inverse() * diff));
    } else {
        double diff = (x - m).value();
        val = -0.5 * (std::log(2.0 * M_PI * cov.value()) +
                      std::pow(diff, 2) / cov.value());
    }
    return std::exp(val);
}

}  // namespace lager::gncpy::math
