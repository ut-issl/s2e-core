/**
 * @file test_runge_kutta.cpp
 * @brief Test codes for RungeKutta class with GoogleTest
 */
#include <gtest/gtest.h>

#include "ode_examples.hpp"

/**
 * @brief Test for constructor from four numbers
 */
TEST(RUNGE_KUTTA, Constructor) {
  libra::ExampleLinearOde linear_ode(0.1);

  libra::Vector<1> state = linear_ode.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);
}

/**
 * @brief Test for integration with nominal linear function
 */
TEST(RUNGE_KUTTA, IntegrateLinear) {
  double step_width_s = 0.1;
  libra::ExampleLinearOde linear_ode(0.1);

  libra::Vector<1> state = linear_ode.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    linear_ode.Integrate();
  }
  state = linear_ode.GetState();
  double estimated_result = step_width_s * step_num;

  EXPECT_NEAR(estimated_result, state[0], 1e-6);
}
