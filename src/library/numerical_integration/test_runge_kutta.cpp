/**
 * @file test_runge_kutta.cpp
 * @brief Test codes for RungeKutta class with GoogleTest
 */
#include <gtest/gtest.h>

#include "ode_examples.hpp"
#include "runge_kutta_4.hpp"
#include "runge_kutta_fehlberg.hpp"

/**
 * @brief Test for constructor
 */
TEST(RUNGE_KUTTA, Constructor) {
  libra::ExampleLinearOde ode;
  libra::RungeKutta4<1> linear_ode(0.1, ode);

  libra::Vector<1> state = linear_ode.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);
}

/**
 * @brief Test for integration with nominal linear function with RK4
 */
TEST(RUNGE_KUTTA, IntegrateLinearRk4) {
  double step_width_s = 0.1;
  libra::ExampleLinearOde ode;
  libra::RungeKutta4<1> rk4_ode(0.1, ode);

  libra::Vector<1> state = rk4_ode.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    rk4_ode.Integrate();
  }
  state = rk4_ode.GetState();
  double estimated_result = step_width_s * step_num;

  EXPECT_NEAR(estimated_result, state[0], 1e-6);
}

/**
 * @brief Test for integration with nominal linear function with RKF
 */
TEST(RUNGE_KUTTA, IntegrateLinearRkf) {
  double step_width_s = 0.1;
  libra::ExampleLinearOde ode;
  libra::RungeKuttaFehlberg<1> rkf_ode(0.1, ode);

  libra::Vector<1> state = rkf_ode.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    rkf_ode.Integrate();
  }
  state = rkf_ode.GetState();
  double estimated_result = step_width_s * step_num;

  EXPECT_NEAR(estimated_result, state[0], 1e-6);
}

/**
 * @brief Test for integration with quadratic function with RK4
 */
TEST(RUNGE_KUTTA, IntegrateQuadraticRk4) {
  double step_width_s = 0.1;
  libra::ExampleQuadraticOde ode;
  libra::RungeKutta4<1> rk4_ode(0.1, ode);

  libra::Vector<1> state = rk4_ode.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    rk4_ode.Integrate();
  }
  state = rk4_ode.GetState();
  double estimated_result = (step_width_s * step_num) * (step_width_s * step_num);

  EXPECT_NEAR(estimated_result, state[0], 1e-6);
}

/**
 * @brief Test for integration with quadratic function with RKF
 */
TEST(RUNGE_KUTTA, IntegrateQuadraticRkf) {
  double step_width_s = 0.1;
  libra::ExampleQuadraticOde ode;
  libra::RungeKuttaFehlberg<1> rkf_ode(0.1, ode);

  libra::Vector<1> state = rkf_ode.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    rkf_ode.Integrate();
  }
  state = rkf_ode.GetState();
  double estimated_result = (step_width_s * step_num) * (step_width_s * step_num);

  EXPECT_NEAR(estimated_result, state[0], 1e-6);
}

/**
 * @brief Test for integration with 1D position and velocity function with RK4
 */
TEST(RUNGE_KUTTA, Integrate1dPositionVelocityRk4) {
  double step_width_s = 0.1;
  libra::Example1dPositionVelocityOde ode;
  libra::RungeKutta4<2> rk4_ode(0.1, ode);

  libra::Vector<2> initial_state(0.0);
  initial_state[0] = 0.0;
  initial_state[1] = 0.1;
  rk4_ode.SetState(0.0, initial_state);

  libra::Vector<2> state = rk4_ode.GetState();
  EXPECT_DOUBLE_EQ(initial_state[0], state[0]);
  EXPECT_DOUBLE_EQ(initial_state[1], state[1]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    rk4_ode.Integrate();
  }
  state = rk4_ode.GetState();
  libra::Vector<2> estimated_result(0.0);
  estimated_result[0] = (step_width_s * step_num) * initial_state[1] + initial_state[0];
  estimated_result[1] = initial_state[1];

  EXPECT_NEAR(estimated_result[0], state[0], 1e-6);
  EXPECT_NEAR(estimated_result[1], state[1], 1e-6);
}

/**
 * @brief Test for integration with 1D position and velocity function with RKF
 */
TEST(RUNGE_KUTTA, Integrate1dPositionVelocityRkf) {
  double step_width_s = 0.1;
  libra::Example1dPositionVelocityOde ode;
  libra::RungeKuttaFehlberg<2> rkf_ode(0.1, ode);

  libra::Vector<2> initial_state(0.0);
  initial_state[0] = 0.0;
  initial_state[1] = 0.1;
  rkf_ode.SetState(0.0, initial_state);

  libra::Vector<2> state = rkf_ode.GetState();
  EXPECT_DOUBLE_EQ(initial_state[0], state[0]);
  EXPECT_DOUBLE_EQ(initial_state[1], state[1]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    rkf_ode.Integrate();
  }
  state = rkf_ode.GetState();
  libra::Vector<2> estimated_result(0.0);
  estimated_result[0] = (step_width_s * step_num) * initial_state[1] + initial_state[0];
  estimated_result[1] = initial_state[1];

  EXPECT_NEAR(estimated_result[0], state[0], 1e-6);
  EXPECT_NEAR(estimated_result[1], state[1], 1e-6);
}
