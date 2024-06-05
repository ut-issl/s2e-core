/**
 * @file attitude_with_cantilever_vibration.cpp
 * @brief Class to calculate spacecraft attitude with Runge-Kutta method
 */
#include "attitude_with_cantilever_vibration.hpp"

#include <logger/log_utility.hpp>
#include <utilities/macros.hpp>

AttitudeWithCantileverVibration::AttitudeWithCantileverVibration(
    const libra::Vector<3>& angular_velocity_b_rad_s, const libra::Quaternion& quaternion_i2b, const libra::Matrix<3, 3>& inertia_tensor_kgm2,
    const libra::Matrix<3, 3>& inertia_tensor_cantilever_kgm2, const double damping_ratio_cantilever,
    const double intrinsic_angular_velocity_cantilever_rad_s, const libra::Vector<3>& torque_b_Nm, const double propagation_step_s,
    const std::string& simulation_object_name)
    : Attitude(inertia_tensor_kgm2, simulation_object_name),
      numerical_integrator_(propagation_step_s, attitude_ode_, libra::numerical_integration::NumericalIntegrationMethod::kRk4) {
  angular_velocity_b_rad_s_ = angular_velocity_b_rad_s;
  quaternion_i2b_ = quaternion_i2b;
  torque_b_Nm_ = torque_b_Nm;
  propagation_step_s_ = propagation_step_s;
  current_propagation_time_s_ = 0.0;
  angular_momentum_reaction_wheel_b_Nms_ = libra::Vector<3>(0.0);
  previous_inertia_tensor_kgm2_ = inertia_tensor_kgm2_;
  inertia_tensor_cantilever_kgm2_ = inertia_tensor_cantilever_kgm2;
  attenuation_coefficient_ = 2 * damping_ratio_cantilever * intrinsic_angular_velocity_cantilever_rad_s;
  spring_coefficient_ = pow(intrinsic_angular_velocity_cantilever_rad_s, 2.0);
  inverse_inertia_tensor_ = CalcInverseMatrix(inertia_tensor_kgm2_);
  inverse_equivalent_inertia_tensor_cantilever_ = CalcInverseMatrix(inertia_tensor_kgm2_ - inertia_tensor_cantilever_kgm2_) * inertia_tensor_kgm2_;
  CalcAngularMomentum();
  SetOdeParameters();
}

AttitudeWithCantileverVibration::~AttitudeWithCantileverVibration() {}

std::string AttitudeWithCantileverVibration::GetLogHeader() const {
  std::string str_tmp = Attitude::GetLogHeader();

  str_tmp += WriteVector("euler_angular_cantilever", "c", "rad", 3);
  str_tmp += WriteVector("angular_velocity_cantilever", "c", "rad/s", 3);

  return str_tmp;
}

std::string AttitudeWithCantileverVibration::GetLogValue() const {
  std::string str_tmp = Attitude::GetLogValue();

  str_tmp += WriteVector(euler_angular_cantilever_rad_);
  str_tmp += WriteVector(angular_velocity_cantilever_rad_s_);

  return str_tmp;
}

void AttitudeWithCantileverVibration::SetParameters(const MonteCarloSimulationExecutor& mc_simulator) {
  Attitude::SetParameters(mc_simulator);
  GetInitializedMonteCarloParameterVector(mc_simulator, "angular_velocity_b_rad_s", angular_velocity_b_rad_s_);

  // TODO: Consider the following calculation is needed here?
  current_propagation_time_s_ = 0.0;
  angular_momentum_reaction_wheel_b_Nms_ = libra::Vector<3>(0.0);  //!< Consider how to handle this variable
  CalcAngularMomentum();
}

void AttitudeWithCantileverVibration::SetOdeParameters() {
  attitude_ode_.SetAttenuationCoefficient(attenuation_coefficient_);
  attitude_ode_.SetSpringCoefficient(spring_coefficient_);
  attitude_ode_.SetTorque_b_Nm(torque_b_Nm_);
  attitude_ode_.SetTorqueInertiaTensor_b_Nm(torque_inertia_tensor_b_Nm_);
  attitude_ode_.SetAngularMomentumReactionWheel_b_Nms(angular_momentum_reaction_wheel_b_Nms_);
  attitude_ode_.SetInverseInertiaTensor(inverse_inertia_tensor_);
  attitude_ode_.SetPreviousInertiaTensor_kgm2(previous_inertia_tensor_kgm2_);
  attitude_ode_.SetInertiaTensorCantilever_kgm2(inertia_tensor_cantilever_kgm2_);
  attitude_ode_.SetInverseEquivalentInertiaTensorCantilever(inverse_equivalent_inertia_tensor_cantilever_);
}

void AttitudeWithCantileverVibration::Propagate(const double end_time_s) {
  if (!is_calc_enabled_) return;

  libra::Matrix<3, 3> dot_inertia_tensor =
      (1.0 / (end_time_s - current_propagation_time_s_)) * (inertia_tensor_kgm2_ - previous_inertia_tensor_kgm2_);
  torque_inertia_tensor_b_Nm_ = dot_inertia_tensor * angular_velocity_b_rad_s_;
  inverse_inertia_tensor_ = CalcInverseMatrix(inertia_tensor_kgm2_);
  SetOdeParameters();

  libra::Vector<13> state = SetStateFromPhysicalQuantities();
  numerical_integrator_.GetIntegrator()->SetState(propagation_step_s_, state);
  while (end_time_s - current_propagation_time_s_ - propagation_step_s_ > 1.0e-6) {
    numerical_integrator_.GetIntegrator()->Integrate();
    current_propagation_time_s_ += propagation_step_s_;
  }
  numerical_integrator_.GetIntegrator()->SetState(end_time_s - current_propagation_time_s_, numerical_integrator_.GetIntegrator()->GetState());
  numerical_integrator_.GetIntegrator()->Integrate();
  SetPhysicalQuantitiesFromState(numerical_integrator_.GetIntegrator()->GetState());

  // Update information
  current_propagation_time_s_ = end_time_s;
  previous_inertia_tensor_kgm2_ = inertia_tensor_kgm2_;
  CalcAngularMomentum();
}

libra::Vector<13> AttitudeWithCantileverVibration::SetStateFromPhysicalQuantities() {
  libra::Vector<13> state;
  for (int i = 0; i < 3; i++) {
    state[i] = angular_velocity_b_rad_s_[i];
  }
  for (int i = 0; i < 3; i++) {
    state[i + 3] = angular_velocity_cantilever_rad_s_[i];
  }
  for (int i = 0; i < 4; i++) {
    state[i + 6] = quaternion_i2b_[i];
  }
  for (int i = 0; i < 3; i++) {
    state[i + 10] = euler_angular_cantilever_rad_[i];
  }
  return state;
}

void AttitudeWithCantileverVibration::SetPhysicalQuantitiesFromState(libra::Vector<13> state) {
  for (int i = 0; i < 3; i++) {
    angular_velocity_b_rad_s_[i] = state[i];
  }
  for (int i = 0; i < 3; i++) {
    angular_velocity_cantilever_rad_s_[i] = state[i + 3];
  }
  for (int i = 0; i < 4; i++) {
    quaternion_i2b_[i] = state[i + 6];
  }
  quaternion_i2b_.Normalize();
  for (int i = 0; i < 3; i++) {
    euler_angular_cantilever_rad_[i] = state[i + 10];
  }
}
