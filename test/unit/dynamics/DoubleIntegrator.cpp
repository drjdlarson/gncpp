#include <iostream>
#include <sstream>

#include <gtest/gtest.h>
#include <cereal/archives/portable_binary.hpp>

#include <gncpy/dynamics/Parameters.h>
#include <gncpy/dynamics/DoubleIntegrator.h>
#include <gncpy/math/Matrix.h>
#include <gncpy/math/Vector.h>


TEST(DoubleInt, Propagate) {
    double dt = 0.1;
    lager::gncpy::dynamics::DoubleIntegrator dyn(dt);
    lager::gncpy::matrix::Vector xk({0., 0., 1., 0.});

    for(uint16_t kk = 0; kk < 10; kk++) {
        double timestep = kk * dt;
        std::cout << "t = " << timestep << ": ";

        xk = dyn.propagateState(timestep, xk);
        for(auto const& x : xk) {
            std::cout << x << " ";
        }
        std::cout << std::endl;
    }

    EXPECT_DOUBLE_EQ(xk(0), 1.);

    SUCCEED();
}


TEST(DoubleInt, Serialize) {
    double dt = 0.1;
    lager::gncpy::dynamics::DoubleIntegrator dyn(dt);
    dyn.setControlModel([](double t, const lager::gncpy::dynamics::ControlParams* params) -> lager::gncpy::matrix::Matrix<double> {
        return lager::gncpy::matrix::identity<double>(4);
    });

    const char* filtState = dyn.saveFilterState();
    auto dyn2 = lager::gncpy::dynamics::DoubleIntegrator<double>::loadFilterState(filtState);

    EXPECT_DOUBLE_EQ(dyn.dt(), dyn2.dt());
    EXPECT_EQ(dyn.hasControlModel(), dyn2.hasControlModel());
    EXPECT_EQ(dyn.hasStateConstraint(), dyn2.hasStateConstraint());

    SUCCEED();
}