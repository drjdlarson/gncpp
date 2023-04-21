#pragma once
#include <cmath>
#include <functional>
#include <vector>

#include "gncpy/math/Matrix.h"
#include "gncpy/math/Vector.h"

namespace lager::gncpy::math {

/**
 * @brief Get the gradient vector of the function
 *
 * This numerically calculates the gradient of \f$ f(\vec{x})
 * \f$ using a midpoint rule, i.e.
 *
 * \f[
 *      \nabla_{\vec{x}} f = \begin{bmatrix}
 *          \frac{\partial f}{\partial x_0} \\
 *          \frac{\partial f}{\partial x_1} \\
 *          \vdots \\
 *          \frac{\partial f}{\partial x_N}
 *      \end{bmatrix}
 * \f]
 *
 * @param x Vector to evaluate the jacobian at
 * @param fnc Function to take the jacobian of, must take in a vector and return
 * type T
 * @return matrix::Vector<T> Jacobian/gradient vector
 */
template <typename T>
matrix::Vector<T> getGradient(
    const matrix::Vector<T>& x,
    std::function<T(const matrix::Vector<T>&)> const& fnc) {
    const double step = 1e-7;
    const T invStep2 = 1. / (2. * step);

    std::vector<T> data;
    matrix::Vector xR(x);
    matrix::Vector xL(x);
    for (uint8_t ii = 0; ii < x.size(); ii++) {
        xR(ii) += step;
        xL(ii) -= step;

        data.emplace_back((fnc(xR) - fnc(xL)) * invStep2);

        // reset for next round
        xR(ii) -= step;
        xL(ii) += step;
    }

    return matrix::Vector<T>(data.size(), data);
}

/**
 * @brief Calculate the Jacobian matrix.
 *
 * This numerically calculates the Jacobian matrix for the given vector
 * function by using getGradient. To make this easier to specify, an std::vector
 * of function objects is taken in by this function (i.e. each entry in the
 * std::vector is one element in the function vector, \f$ f_i \f$, that takes in
 * a vector and returns a type T).
 *
 * \f[
 *      J = \begin{bmatrix}
 *              \nabla^T f_0 \\
 *              \vdots \\
 *              \nabla^T f_m
 *          \end{bmatrix}
 *          = \begin{bmatrix}
 *              \frac{\partial f_0}{\partial x_0} & \dots & \frac{\partial
 * f_0}{\partial x_N} \\
 *              \vdots & \ddots & \vdots \\
 *              \frac{\partial f_m}{\partial x_0} & \dots & \frac{\partial
 * f_m}{\partial x_N}
 *          \end{bmatrix}
 * \f]
 *
 *
 * @param x The vector to take the Jacobian about
 * @param fncLst List of functions to take the jacobian of
 * @return matrix::Matrix<T> Jacobian matrix
 */
template <typename T>
matrix::Matrix<T> getJacobian(
    const matrix::Vector<T>& x,
    const std::vector<std::function<T(const matrix::Vector<T>&)>>& fncLst) {
    std::vector<T> data;
    for (auto const& f : fncLst) {
        for (auto& val : getGradient<T>(x, f)) {
            data.emplace_back(val);
        }
    }

    return matrix::Matrix<T>(fncLst.size(), x.size(), data);
}

/**
 * @brief Calculate the Jacobian matrix.
 *
 * This is an alternative form to calculate the Jacobian of a vector valued
 * function. Here a single function is specified that takes a vector and returns
 * a vector (instead of a list of functions returning individual elements of the
 * resulting vector). To save a function evaluation, the number of elements in
 * the vector returned by the given function must also be specified.
 *
 * @param x Point to take the Jacobian about
 * @param fnc Vector valued function to take the Jacobian of
 * @param numFunOutputs Number of elements in the vector returned by the
 * supplied function
 * @return matrix::Matrix<T> Jacobian matrix
 */
template <typename T>
matrix::Matrix<T> getJacobian(
    const matrix::Vector<T>& x,
    std::function<matrix::Vector<T>(const matrix::Vector<T>&)> const& fnc,
    size_t numFunOutputs) {
    std::vector<T> data(numFunOutputs * x.size());
    size_t ind = 0;

    for (size_t row = 0; row < numFunOutputs; row++) {
        auto fi = [&fnc, row](const matrix::Vector<T>& x_) {
            return fnc(x_)(row);
        };
        for (auto& val : getGradient<double>(x, fi)) {
            data[ind] = val;
            ind++;
        }
    }

    return matrix::Matrix(numFunOutputs, x.size(), data);
}

/**
 * @brief Calculate the value of a multi-variate Gaussian PDF
 *
 * @param x Vector to evaluate the PDF at
 * @param m Mean of the distribution
 * @param cov Covariance matrix of the distribution
 * @return T PDF value
 */
template <typename T>
T calcGaussianPDF(const matrix::Vector<T>& x, const matrix::Vector<T>& m,
                  const matrix::Matrix<T>& cov) {
    uint8_t nDim = x.size();
    T val;
    if (nDim > 1) {
        matrix::Vector<T> diff = x - m;
        val =
            -0.5 * (static_cast<T>(nDim) * std::log(static_cast<T>(2) * M_PI) +
                    std::log(cov.determinant()) +
                    (diff.transpose() * cov.inverse() * diff).toScalar());
    } else {
        T diff = (x - m).toScalar();
        val = -0.5 * (std::log(static_cast<T>(2) * M_PI * cov.toScalar()) +
                      std::pow(diff, 2) / cov.toScalar());
    }
    return std::exp(val);
}

/**
 * @brief Numerical integration using a basic Runge-Kutta method.
 *
 * This implements the classic (RK4) version with a fixed step size according to
 *
 * \f{align}{
 *      x_{n+1} &= x_n + \frac{1}{6}(k_0 + 2k_1 + 2k_2 + k_4)dt \\
 *      t_{n+t} &= t_n + dt \\
 *      k_0 &= f(t_n, x_n) \\
 *      k_1 &= f(t_n + \frac{dt}{2}, x_n + dt\frac{k_0}{2}) \\
 *      k_2 &= f(t_n + \frac{dt}{2}, x_n + dt\frac{k_1}{2}) \\
 *      k_3 &= f(t_n + dt, x_n + dt k_2)
 * \f}
 *
 * for \f$n = 0, 1, \dots, \f$ and \f$\frac{dx}{dt} = f(t, x)\f$. In this
 * function, \f$n=0\f$ and the \f$n+1\f$ output is calculated.
 *
 * @tparam return_t Data type for the return value
 * @tparam state_t Data type for the state (i.e. matrix::Vector, float, double)
 * @tparam step_t Underlying data type for the time step
 * @param t0 Initial time
 * @param x0 Initial state
 * @param dt Integration time
 * @param fnc Function to integrate
 */
template <typename return_t, typename state_t, typename step_t>
return_t rungeKutta4(
    step_t t0, const state_t& x0, step_t dt,
    std::function<return_t(step_t, const state_t&)> const& fnc) {
    step_t dtHalf = static_cast<step_t>(0.5) * dt;
    step_t tHalf = t0 + dtHalf;
    return_t k0 = fnc(t0, x0);
    return_t k1 = fnc(tHalf, x0 + dtHalf * k0);
    return_t k2 = fnc(tHalf, x0 + dtHalf * k1);
    return_t k3 = fnc(t0 + dt, x0 + dt * k2);

    return x0 + dt / static_cast<step_t>(6.0) *
                    (k0 + static_cast<step_t>(2) * (k1 + k2) + k3);
}

}  // namespace lager::gncpy::math
