/**
 * @file attitude_with_cantilever_vibration.cpp
 * @brief Class to calculate spacecraft attitude with cantilever vibration
 */
#include "attitude_with_cantilever_vibration.hpp"

#include <cassert>
#include <logger/log_utility.hpp>
#include <utilities/macros.hpp>

AttitudeWithCantileverVibration::AttitudeWithCantileverVibration(
    const math::Vector<3>& angular_velocity_b_rad_s, const math::Quaternion& quaternion_i2b, const math::Matrix<3, 3>& inertia_tensor_kgm2,
    const math::Matrix<3, 3>& inertia_tensor_cantilever_kgm2, const double damping_ratio_cantilever,
    const double intrinsic_angular_velocity_cantilever_rad_s, const math::Vector<3>& torque_b_Nm, const double propagation_step_s,
    const std::string& simulation_object_name)
    : Attitude(inertia_tensor_kgm2, simulation_object_name),
      numerical_integrator_(propagation_step_s, attitude_ode_,
                            libra::numerical_integration::NumericalIntegrationMethod::kRk4) {  //!< TODO: Set NumericalIntegrationMethod in *.ini file
  angular_velocity_b_rad_s_ = angular_velocity_b_rad_s;
  quaternion_i2b_ = quaternion_i2b;
  torque_b_Nm_ = torque_b_Nm;
  propagation_step_s_ = propagation_step_s;
  current_propagation_time_s_ = 0.0;
  angular_momentum_reaction_wheel_b_Nms_ = math::Vector<3>(0.0);

  attitude_ode_.SetInertiaTensorCantilever_kgm2(inertia_tensor_cantilever_kgm2);
  attitude_ode_.SetPreviousInertiaTensor_kgm2(inertia_tensor_kgm2_);
  double attenuation_coefficient = 2 * damping_ratio_cantilever * intrinsic_angular_velocity_cantilever_rad_s;
  attitude_ode_.SetAttenuationCoefficient(attenuation_coefficient);
  double spring_coefficient = pow(intrinsic_angular_velocity_cantilever_rad_s, 2.0);
  attitude_ode_.SetSpringCoefficient(spring_coefficient);
  attitude_ode_.SetInverseInertiaTensor(CalcInverseMatrix(inertia_tensor_kgm2_));
  math::Matrix<3, 3> inverse_equivalent_inertia_tensor_cantilever =
      CalcInverseMatrix(inertia_tensor_kgm2_ - inertia_tensor_cantilever_kgm2) * inertia_tensor_kgm2_;
  attitude_ode_.SetInverseEquivalentInertiaTensorCantilever(inverse_equivalent_inertia_tensor_cantilever);
  attitude_ode_.SetTorque_b_Nm(torque_b_Nm_);
  attitude_ode_.SetAngularMomentumReactionWheel_b_Nms(angular_momentum_reaction_wheel_b_Nms_);

  CalcAngularMomentum();
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
  angular_momentum_reaction_wheel_b_Nms_ = math::Vector<3>(0.0);  //!< TODO: Consider how to handle this variable
  CalcAngularMomentum();
}

void AttitudeWithCantileverVibration::Propagate(const double end_time_s) {
  if (!is_calc_enabled_) return;

  math::Matrix<3, 3> previous_inertia_tensor_kgm2 = attitude_ode_.GetPreviousInertiaTensor_kgm2();
  assert(end_time_s - current_propagation_time_s_ > 1e-6);
  math::Matrix<3, 3> dot_inertia_tensor = (1.0 / (end_time_s - current_propagation_time_s_)) * (inertia_tensor_kgm2_ - previous_inertia_tensor_kgm2);
  math::Vector<3> torque_inertia_tensor_b_Nm = dot_inertia_tensor * angular_velocity_b_rad_s_;
  attitude_ode_.SetTorqueInertiaTensor_b_Nm(torque_inertia_tensor_b_Nm);
  attitude_ode_.SetInverseInertiaTensor(CalcInverseMatrix(inertia_tensor_kgm2_));
  attitude_ode_.SetTorque_b_Nm(torque_b_Nm_);
  attitude_ode_.SetAngularMomentumReactionWheel_b_Nms(angular_momentum_reaction_wheel_b_Nms_);

  math::Vector<13> state = attitude_ode_.SetStateFromPhysicalQuantities(angular_velocity_b_rad_s_, angular_velocity_cantilever_rad_s_,
                                                                         quaternion_i2b_, euler_angular_cantilever_rad_);
  numerical_integrator_.GetIntegrator()->SetState(propagation_step_s_, state);
  while (end_time_s - current_propagation_time_s_ - propagation_step_s_ > 1.0e-6) {
    numerical_integrator_.GetIntegrator()->Integrate();
    current_propagation_time_s_ += propagation_step_s_;
  }
  numerical_integrator_.GetIntegrator()->SetState(end_time_s - current_propagation_time_s_, numerical_integrator_.GetIntegrator()->GetState());
  numerical_integrator_.GetIntegrator()->Integrate();
  attitude_ode_.SetPhysicalQuantitiesFromState(numerical_integrator_.GetIntegrator()->GetState(), angular_velocity_b_rad_s_,
                                               angular_velocity_cantilever_rad_s_, quaternion_i2b_, euler_angular_cantilever_rad_);
  quaternion_i2b_.Normalize();

  // Update information
  current_propagation_time_s_ = end_time_s;
  attitude_ode_.SetPreviousInertiaTensor_kgm2(inertia_tensor_kgm2_);
  CalcAngularMomentum();
}
