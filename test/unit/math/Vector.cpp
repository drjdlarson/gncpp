#include "gncpy/math/Vector.h"

#include <gtest/gtest.h>

TEST(VectorTest, Index) {
    lager::gncpy::matrix::Vector v({2, 3});

    EXPECT_EQ(v(0), 2);
    EXPECT_EQ(v(1), 3);

    SUCCEED();
}

TEST(VectorTest, Add) {
    lager::gncpy::matrix::Vector v1({2, 3});
    lager::gncpy::matrix::Vector v2({4, 5});

    lager::gncpy::matrix::Vector<int> res = v1 + v2;
    lager::gncpy::matrix::Vector exp({6, 8});

    EXPECT_EQ(exp(0), res(0));
    EXPECT_EQ(exp(1), res(1));

    SUCCEED();
}

TEST(VectorTest, Subtract) {
    lager::gncpy::matrix::Vector v1({2, 3});
    lager::gncpy::matrix::Vector v2({1, 1});

    lager::gncpy::matrix::Vector<int> res = v1 - v2;
    lager::gncpy::matrix::Vector exp({1, 2});

    EXPECT_EQ(exp(0), res(0));
    EXPECT_EQ(exp(1), res(1));

    SUCCEED();
}

TEST(VectorTest, serialize) {
    lager::gncpy::matrix::Vector<double> v({-2.3, 1, 8.9, -9.2});

    std::cout << "Original class:\n" << v.toJSON() << std::endl;
    std::stringstream classState = v.saveClassState();

    auto v2 = lager::gncpy::matrix::Vector<double>::loadClass(classState);
    std::cout << "Loaded class:\n" << v2.toJSON() << std::endl;

    EXPECT_EQ(v.numRows(), v2.numRows());
    EXPECT_EQ(v.numCols(), v2.numCols());

    for (size_t ii = 0; ii < v.size(); ii++) {
        EXPECT_DOUBLE_EQ(v(ii), v2(ii));
    }

    SUCCEED();
}