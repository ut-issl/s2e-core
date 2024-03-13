/**
 * @file test_interpolation_orbit.cpp
 * @brief Test codes for InterpolationOrbit class with GoogleTest
 */
#include <gtest/gtest.h>

#include <cmath>

#include "interpolation_orbit.hpp"

/**
 * @brief Test for Constructor function
 */
TEST(InterpolationOrbit, Constructor) {
  size_t degree = 9;
  InterpolationOrbit interpolation_orbit(degree);

  EXPECT_EQ(degree, interpolation_orbit.GetDegree());
  for (size_t i = 0; i < degree; i++) {
    EXPECT_DOUBLE_EQ(-1.0, interpolation_orbit.GetTimeList()[i]);
  }
}

/**
 * @brief Test for PushAndPop function
 */
TEST(InterpolationOrbit, PushAndPop) {
  size_t degree = 5;
  InterpolationOrbit interpolation_orbit(degree);

  EXPECT_EQ(degree, interpolation_orbit.GetDegree());
  for (size_t i = 0; i < degree; i++) {
    double time = (double)i;
    libra::Vector<3> position{i * 2.0};
    bool ret = interpolation_orbit.PushAndPopData(time, position);
    EXPECT_TRUE(ret);
  }
  for (size_t i = 0; i < degree; i++) {
    EXPECT_DOUBLE_EQ((double)i, interpolation_orbit.GetTimeList()[i]);
    for (size_t axis = 0; axis < 3; axis++) {
      EXPECT_DOUBLE_EQ(2.0 * i, interpolation_orbit.GetPositionDataList(axis)[i]);
    }
  }

  // False test
  double time = 2.0;
  libra::Vector<3> position{-100.0};
  bool ret = interpolation_orbit.PushAndPopData(time, position);
  EXPECT_FALSE(ret);
}
