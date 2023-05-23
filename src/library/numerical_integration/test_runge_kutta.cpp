/**
 * @file test_runge_kutta.cpp
 * @brief Test codes for RungeKutta class with GoogleTest
 */
#include <gtest/gtest.h>

#include "runge_kutta_4.hpp"

/**
 * @brief Test for constructor from four numbers
 */
TEST(RUNGE_KUTTA, Constructor) {
  libra::RungeKutta4<1> rk(0.1);

  libra::Vector<1> state = rk.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);
}

/**
 * @brief Test for integration with nominal linear function
 */
TEST(RUNGE_KUTTA, IntegrateLinear) {
  double step_width_s = 0.1;
  libra::RungeKutta4<1> rk(step_width_s);
  rk.SetParameters();

  libra::Vector<1> state = rk.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    rk.Integrate();
  }
  state = rk.GetState();
  double estimated_result = step_width_s * step_num;

  EXPECT_NEAR(estimated_result, state[0], 1e-6);
}

/**
 * @brief Test for integration
 */
TEST(RUNGE_KUTTA, IntegrateSin) {
  double step_width_s = 0.1;
  libra::RungeKutta4<1> rk(step_width_s);
  rk.SetParameters();

  libra::Vector<1> state = rk.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    rk.Integrate();
  }
  state = rk.GetState();
  double estimated_result = step_width_s * step_num;

  EXPECT_NEAR(estimated_result, state[0], 1e-6);
}
