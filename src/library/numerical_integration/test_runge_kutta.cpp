/**
 * @file test_runge_kutta.cpp
 * @brief Test codes for RungeKutta class with GoogleTest
 */
#include <gtest/gtest.h>

#include "../orbit/kepler_orbit.hpp"
#include "numerical_integrator_manager.hpp"
#include "ode_examples.hpp"
#include "runge_kutta_4.hpp"
#include "runge_kutta_fehlberg.hpp"

/**
 * @brief Test for constructor
 */
TEST(RUNGE_KUTTA, Constructor) {
  libra::numerical_integrator::ExampleLinearOde ode;
  libra::numerical_integrator::RungeKutta4<1> linear_ode(0.1, ode);

  libra::Vector<1> state = linear_ode.GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);
}

/**
 * @brief Test for integration with nominal linear function with RK4
 */
TEST(RUNGE_KUTTA, IntegrateLinearRk4) {
  double step_width_s = 0.1;
  libra::numerical_integrator::ExampleLinearOde ode;
  libra::numerical_integrator::RungeKutta4<1> rk4_ode(step_width_s, ode);

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
  libra::numerical_integrator::ExampleLinearOde ode;
  libra::numerical_integrator::RungeKuttaFehlberg<1> rkf_ode(step_width_s, ode);

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
 * @brief Test for integration with nominal linear function with NumericalIntegratorManager/RK4
 */
TEST(RUNGE_KUTTA, IntegrateLinearNumericalIntegratorManagerRk4) {
  double step_width_s = 0.1;
  libra::numerical_integrator::ExampleLinearOde ode;
  libra::numerical_integrator::NumericalIntegratorManager<1> numerical_integrator(step_width_s, ode);

  libra::Vector<1> state = numerical_integrator.GetIntegrator()->GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    numerical_integrator.GetIntegrator()->Integrate();
  }
  state = numerical_integrator.GetIntegrator()->GetState();
  double estimated_result = step_width_s * step_num;

  EXPECT_NEAR(estimated_result, state[0], 1e-6);
}

/**
 * @brief Test for integration with nominal linear function with NumericalIntegratorManager/RK4
 */
TEST(RUNGE_KUTTA, IntegrateLinearNumericalIntegratorManagerRkf) {
  double step_width_s = 0.1;
  libra::numerical_integrator::ExampleLinearOde ode;
  libra::numerical_integrator::NumericalIntegratorManager<1> numerical_integrator(step_width_s, ode,
                                                                                  libra::numerical_integrator::NumericalIntegrationMethod::kRkf);

  libra::Vector<1> state = numerical_integrator.GetIntegrator()->GetState();
  EXPECT_DOUBLE_EQ(0.0, state[0]);

  size_t step_num = 10000;
  for (size_t i = 0; i < step_num; i++) {
    numerical_integrator.GetIntegrator()->Integrate();
  }
  state = numerical_integrator.GetIntegrator()->GetState();
  double estimated_result = step_width_s * step_num;

  EXPECT_NEAR(estimated_result, state[0], 1e-6);
}

/**
 * @brief Test for integration with quadratic function with RK4
 */
TEST(RUNGE_KUTTA, IntegrateQuadraticRk4) {
  double step_width_s = 0.1;
  libra::numerical_integrator::ExampleQuadraticOde ode;
  libra::numerical_integrator::RungeKutta4<1> rk4_ode(step_width_s, ode);

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
  libra::numerical_integrator::ExampleQuadraticOde ode;
  libra::numerical_integrator::RungeKuttaFehlberg<1> rkf_ode(step_width_s, ode);

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
  libra::numerical_integrator::Example1dPositionVelocityOde ode;
  libra::numerical_integrator::RungeKutta4<2> rk4_ode(step_width_s, ode);

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
  libra::numerical_integrator::Example1dPositionVelocityOde ode;
  libra::numerical_integrator::RungeKuttaFehlberg<2> rkf_ode(step_width_s, ode);

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

/**
 * @brief Accuracy comparison between RK4 and RKF for integration with 2D two body orbit with small eccentricity
 */
TEST(RUNGE_KUTTA, Integrate2dTwoBodyOrbitSmallEccentricity) {
  double step_width_s = 0.1;
  libra::numerical_integrator::Example2dTwoBodyOrbitOde ode;
  libra::numerical_integrator::RungeKutta4<4> rk4_ode(step_width_s, ode);
  libra::numerical_integrator::RungeKuttaFehlberg<4> rkf_ode(step_width_s, ode);

  libra::Vector<4> initial_state(0.0);
  const double eccentricity = 0.1;
  initial_state[0] = 1.0 - eccentricity;
  initial_state[1] = 0.0;
  initial_state[2] = 0.0;
  initial_state[3] = sqrt((1.0 + eccentricity) / (1.0 - eccentricity));
  rk4_ode.SetState(0.0, initial_state);
  rkf_ode.SetState(0.0, initial_state);

  libra::Vector<4> state_rk4 = rk4_ode.GetState();
  libra::Vector<4> state_rkf = rkf_ode.GetState();
  for (size_t i = 0; i < 4; i++) {
    EXPECT_DOUBLE_EQ(initial_state[i], state_rk4[i]);
    EXPECT_DOUBLE_EQ(initial_state[i], state_rkf[i]);
  }

  size_t step_num = 200;
  for (size_t i = 0; i < step_num; i++) {
    rk4_ode.Integrate();
    rkf_ode.Integrate();
  }
  state_rk4 = rk4_ode.GetState();
  state_rkf = rkf_ode.GetState();

  // Estimation by Kepler Orbit calculation
  libra::Vector<3> initial_position(0.0);
  libra::Vector<3> initial_velocity(0.0);

  initial_position[0] = initial_state[0];
  initial_position[1] = initial_state[1];
  initial_velocity[0] = initial_state[2];
  initial_velocity[1] = initial_state[3];
  OrbitalElements oe(1.0, 0.0, initial_position, initial_velocity);
  KeplerOrbit kepler(1.0, oe);
  kepler.CalcOrbit((double)(step_num * step_width_s) / (24.0 * 60.0 * 60.0));

  double error_tolerance = 2e-4;
  EXPECT_NEAR(kepler.GetPosition_i_m()[0], state_rk4[0], error_tolerance);
  EXPECT_NEAR(kepler.GetPosition_i_m()[1], state_rk4[1], error_tolerance);
  EXPECT_NEAR(kepler.GetVelocity_i_m_s()[0], state_rk4[2], error_tolerance);
  EXPECT_NEAR(kepler.GetVelocity_i_m_s()[1], state_rk4[3], error_tolerance);

  error_tolerance = 2e-5;
  EXPECT_NEAR(kepler.GetPosition_i_m()[0], state_rkf[0], error_tolerance);
  EXPECT_NEAR(kepler.GetPosition_i_m()[1], state_rkf[1], error_tolerance);
  EXPECT_NEAR(kepler.GetVelocity_i_m_s()[0], state_rkf[2], error_tolerance);
  EXPECT_NEAR(kepler.GetVelocity_i_m_s()[1], state_rkf[3], error_tolerance);
}

/**
 * @brief Accuracy comparison between RK4 and RKF for integration with 2D two body orbit with high eccentricity
 */
TEST(RUNGE_KUTTA, Integrate2dTwoBodyOrbitLargeEccentricity) {
  double step_width_s = 0.01;
  libra::numerical_integrator::Example2dTwoBodyOrbitOde ode;
  libra::numerical_integrator::RungeKutta4<4> rk4_ode(step_width_s, ode);
  libra::numerical_integrator::RungeKuttaFehlberg<4> rkf_ode(step_width_s, ode);

  libra::Vector<4> initial_state(0.0);
  const double eccentricity = 0.9;
  initial_state[0] = 1.0 - eccentricity;
  initial_state[1] = 0.0;
  initial_state[2] = 0.0;
  initial_state[3] = sqrt((1.0 + eccentricity) / (1.0 - eccentricity));
  rk4_ode.SetState(0.0, initial_state);
  rkf_ode.SetState(0.0, initial_state);

  libra::Vector<4> state_rk4 = rk4_ode.GetState();
  libra::Vector<4> state_rkf = rkf_ode.GetState();
  for (size_t i = 0; i < 4; i++) {
    EXPECT_DOUBLE_EQ(initial_state[i], state_rk4[i]);
    EXPECT_DOUBLE_EQ(initial_state[i], state_rkf[i]);
  }

  size_t step_num = 2000;
  for (size_t i = 0; i < step_num; i++) {
    rk4_ode.Integrate();
    rkf_ode.Integrate();
  }
  state_rk4 = rk4_ode.GetState();
  state_rkf = rkf_ode.GetState();

  // Estimation by Kepler Orbit calculation
  libra::Vector<3> initial_position(0.0);
  libra::Vector<3> initial_velocity(0.0);

  initial_position[0] = initial_state[0];
  initial_position[1] = initial_state[1];
  initial_velocity[0] = initial_state[2];
  initial_velocity[1] = initial_state[3];
  OrbitalElements oe(1.0, 0.0, initial_position, initial_velocity);
  KeplerOrbit kepler(1.0, oe);
  kepler.CalcOrbit((double)(step_num * step_width_s) / (24.0 * 60.0 * 60.0));

  double error_tolerance = 2e-1;
  EXPECT_NEAR(kepler.GetPosition_i_m()[0], state_rk4[0], error_tolerance);
  EXPECT_NEAR(kepler.GetPosition_i_m()[1], state_rk4[1], error_tolerance);
  EXPECT_NEAR(kepler.GetVelocity_i_m_s()[0], state_rk4[2], error_tolerance);
  EXPECT_NEAR(kepler.GetVelocity_i_m_s()[1], state_rk4[3], error_tolerance);

  error_tolerance = 1e-2;
  EXPECT_NEAR(kepler.GetPosition_i_m()[0], state_rkf[0], error_tolerance);
  EXPECT_NEAR(kepler.GetPosition_i_m()[1], state_rkf[1], error_tolerance);
  EXPECT_NEAR(kepler.GetVelocity_i_m_s()[0], state_rkf[2], error_tolerance);
  EXPECT_NEAR(kepler.GetVelocity_i_m_s()[1], state_rkf[3], error_tolerance);
}
