/**
 * @file attitude.cpp
 * @brief Base class for attitude of spacecraft
 */
#include "attitude.hpp"

#include <logger/log_utility.hpp>

Attitude::Attitude(const math::Matrix<3, 3>& inertia_tensor_kgm2, const std::string& simulation_object_name)
    : SimulationObject(simulation_object_name), inertia_tensor_kgm2_(inertia_tensor_kgm2) {
  angular_velocity_b_rad_s_ = math::Vector<3>(0.0);
  quaternion_i2b_ = math::Quaternion(0.0, 0.0, 0.0, 1.0);
  torque_b_Nm_ = math::Vector<3>(0.0);
  angular_momentum_spacecraft_b_Nms_ = math::Vector<3>(0.0);
  angular_momentum_reaction_wheel_b_Nms_ = math::Vector<3>(0.0);
  angular_momentum_total_b_Nms_ = math::Vector<3>(0.0);
  angular_momentum_total_i_Nms_ = math::Vector<3>(0.0);
  angular_momentum_total_Nms_ = 0.0;
  kinetic_energy_J_ = 0.0;
}

std::string Attitude::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteVector("spacecraft_angular_velocity", "b", "rad/s", 3);
  str_tmp += WriteQuaternion("spacecraft_quaternion", "i2b");
  str_tmp += WriteVector("spacecraft_torque", "b", "Nm", 3);
  str_tmp += WriteScalar("spacecraft_total_angular_momentum", "Nms");
  str_tmp += WriteScalar("spacecraft_kinematic_energy", "J");

  return str_tmp;
}

std::string Attitude::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(angular_velocity_b_rad_s_);
  str_tmp += WriteQuaternion(quaternion_i2b_);
  str_tmp += WriteVector(torque_b_Nm_);
  str_tmp += WriteScalar(angular_momentum_total_Nms_);
  str_tmp += WriteScalar(kinetic_energy_J_);

  return str_tmp;
}

void Attitude::SetParameters(const MonteCarloSimulationExecutor& mc_simulator) {
  GetInitializedMonteCarloParameterQuaternion(mc_simulator, "quaternion_i2b", quaternion_i2b_);
}

void Attitude::CalcAngularMomentum(void) {
  angular_momentum_spacecraft_b_Nms_ = inertia_tensor_kgm2_ * angular_velocity_b_rad_s_;
  angular_momentum_total_b_Nms_ = angular_momentum_reaction_wheel_b_Nms_ + angular_momentum_spacecraft_b_Nms_;
  math::Quaternion q_b2i = quaternion_i2b_.Conjugate();
  angular_momentum_total_i_Nms_ = q_b2i.FrameConversion(angular_momentum_total_b_Nms_);
  angular_momentum_total_Nms_ = angular_momentum_total_i_Nms_.CalcNorm();

  kinetic_energy_J_ = 0.5 * math::InnerProduct(angular_momentum_spacecraft_b_Nms_, angular_velocity_b_rad_s_);
}

math::Matrix<4, 4> CalcAngularVelocityMatrix(math::Vector<3> angular_velocity_b_rad_s) {
  math::Matrix<4, 4> angular_velocity_matrix;

  angular_velocity_matrix[0][0] = 0.0f;
  angular_velocity_matrix[0][1] = angular_velocity_b_rad_s[2];
  angular_velocity_matrix[0][2] = -angular_velocity_b_rad_s[1];
  angular_velocity_matrix[0][3] = angular_velocity_b_rad_s[0];
  angular_velocity_matrix[1][0] = -angular_velocity_b_rad_s[2];
  angular_velocity_matrix[1][1] = 0.0f;
  angular_velocity_matrix[1][2] = angular_velocity_b_rad_s[0];
  angular_velocity_matrix[1][3] = angular_velocity_b_rad_s[1];
  angular_velocity_matrix[2][0] = angular_velocity_b_rad_s[1];
  angular_velocity_matrix[2][1] = -angular_velocity_b_rad_s[0];
  angular_velocity_matrix[2][2] = 0.0f;
  angular_velocity_matrix[2][3] = angular_velocity_b_rad_s[2];
  angular_velocity_matrix[3][0] = -angular_velocity_b_rad_s[0];
  angular_velocity_matrix[3][1] = -angular_velocity_b_rad_s[1];
  angular_velocity_matrix[3][2] = -angular_velocity_b_rad_s[2];
  angular_velocity_matrix[3][3] = 0.0f;

  return angular_velocity_matrix;
}
