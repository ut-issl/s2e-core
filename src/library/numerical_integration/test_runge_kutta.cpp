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

/**
 * @brief Test for integration with quadratic function
 */
TEST(RUNGE_KUTTA, IntegrateQuadratic) {
  double step_width_s = 0.1;
  libra::ExampleQuadraticOde ode(0.1);

  libra::Vector<1> state = ode.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    ode.Integrate();
  }
  state = ode.GetState();
  double estimated_result = (step_width_s * step_num) * (step_width_s * step_num);

  EXPECT_NEAR(estimated_result, state[0], 1e-6);
}

/**
 * @brief Test for integration with 1D position and velocity function
 */
TEST(RUNGE_KUTTA, Integrate1dPositionVelocity) {
  double step_width_s = 0.1;
  libra::Example1dPositionVelocityOde ode(0.1);
  libra::Vector<2> initial_state(0.0);
  initial_state[0] = 0.0;
  initial_state[1] = 0.1;
  ode.SetState(0.0, initial_state);

  libra::Vector<2> state = ode.GetState();
  EXPECT_DOUBLE_EQ(initial_state[0], state[0]);
  EXPECT_DOUBLE_EQ(initial_state[1], state[1]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    ode.Integrate();
  }
  state = ode.GetState();
  libra::Vector<2> estimated_result(0.0);
  estimated_result[0] = (step_width_s * step_num) * initial_state[1] + initial_state[0];
  estimated_result[1] = initial_state[1];

  EXPECT_NEAR(estimated_result[0], state[0], 1e-6);
  EXPECT_NEAR(estimated_result[1], state[1], 1e-6);
}
