/*
 * @file flexible_structure.cpp
 * @brief Component emulation of flexible structure
 */

#include "flexible_structure.hpp"

#include <library/math/matrix.hpp>


FlexibleStructure::FlexibleStructure(const unsigned int prescaler, ClockGenerator* clock_generator, const Structure* structure, const Dynamics* dynamics,
                    InstalledComponents* components, double compo_step_s, double propagation_step_s)
  : Component(prescaler, clock_generator), structure_(structure), dynamics_(dynamics), components_(components), compo_step_s_(compo_step_s),
    propagation_step_s_(propagation_step_s), integrator_(new libra::numerical_integration::RungeKutta4(propagation_step_s, ode_)) {
  propagation_time_s_ = 0.0;
  compo_time_s_ = 0.0;
}


FlexibleStructure::~FlexibleStructure() { delete integrator_; }


void FlexibleStructure::MainRoutine(const int time_count) {
  UNUSED(time_count);

  force_to_body_b_N_ = Vector<3>(0.0);
  torque_to_body_b_Nm_ = Vector<3>(0.0);

  double body_mass_kg = structure_->GetKinematicsParameters().GetMass_kg();
  libra::Matrix<3, 3> body_inearia_tensor_b_kgm2 = structure_->GetKinematicsParameters().GetInertiaTensor_b_kgm2();
  libra::Matrix<3, 3> body_inearia_tensor_inverse_b_kgm2 = CalcInverseMatrix(body_inearia_tensor_b_kgm2);

  libra::Quaternion body_quaternion_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  libra::Vector<3> body_angular_velocity_b_rad_s = dynamics_->GetAttitude().GetAngularVelocity_b_rad_s();
  libra::Vector<3> body_angular_velocity_i_m_s = body_quaternion_i2b.InverseFrameConversion(body_angular_velocity_b_rad_s);

  libra::Vector<3> body_generated_force_b_N = components_->GenerateForce_b_N();
  libra::Vector<3> body_generated_torque_b_Nm = components_->GenerateTorque_b_Nm();

  libra::Vector<3> body_angular_acceleration_by_torque_b_rad_s2 =
    body_inearia_tensor_inverse_b_kgm2 * 
    (body_generated_torque_b_Nm - OuterProduct(body_angular_velocity_b_rad_s, body_inearia_tensor_b_kgm2 * body_angular_velocity_b_rad_s));
  libra::Vector<3> body_angular_acceleration_by_torque_i_rad_s2 =
      body_quaternion_i2b.InverseFrameConversion(body_angular_acceleration_by_torque_b_rad_s2);

  libra::Vector<3> body_acceleration_by_force_b_m_s2 = (1.0 / body_mass_kg) * body_generated_force_b_N;
  libra::Vector<3> centrifugal_acceleration_b_m_s2 =
      OuterProduct(body_angular_velocity_i_m_s, OuterProduct(body_angular_velocity_i_m_s, origin_b_m_));
  libra::Vector<3> euler_acceleration_b_m_s2 = OuterProduct(body_angular_acceleration_by_torque_i_rad_s2, origin_b_m_); 
  libra::Vector<3> body_acceleration_by_force_and_inertial_force_b_m_s2 =
      body_acceleration_by_force_b_m_s2 + centrifugal_acceleration_b_m_s2 + euler_acceleration_b_m_s2;

  libra::Matrix<MODE_NUM, MODE_NUM> M;
  libra::Vector<MODE_NUM> c;
  libra::Vector<MODE_NUM> k;
  libra::Vector<MODE_NUM> f;

  for (int i = 0; i < MODE_NUM; i++) {
    M[i][i] = 1.0;

    for (int j = 0; j < MODE_NUM; j++) {
      M[i][j] -= InnerProduct(modes_[i].translation_participation_sqrt_kg, modes_[j].translation_participation_sqrt_kg) / body_mass_kg;
      M[i][j] -= InnerProduct(
          modes_[i].translation_participation_sqrt_kg,
          OuterProduct(body_quaternion_i2b.InverseFrameConversion(body_inearia_tensor_inverse_b_kgm2 * modes_[j].rotation_participation_sqrt_kg_rad_m),
                       origin_b_m_));
      M[i][j] -=
          InnerProduct(modes_[i].rotation_participation_sqrt_kg_rad_m, body_inearia_tensor_inverse_b_kgm2 * modes_[j].rotation_participation_sqrt_kg_rad_m);
    }

    c[i] = 2.0 * modes_[i].damping_ratio * modes_[i].characteristic_angular_frequency_rad_s;
    k[i] = std::pow(modes_[i].characteristic_angular_frequency_rad_s, 2);
    f[i] = 0.0;
    f[i] += InnerProduct(modes_[i].translation_participation_sqrt_kg, body_acceleration_by_force_and_inertial_force_b_m_s2);
    f[i] += InnerProduct(modes_[i].rotation_participation_sqrt_kg_rad_m, body_angular_acceleration_by_torque_b_rad_s2);
  }

  ode_.SetCoeff(M, c, k, f);

  double compo_delta_time = compo_step_s_ * prescaler_;
  compo_time_s_ += compo_delta_time;

  while (propagation_time_s_ < compo_time_s_ - DBL_EPSILON) {
    integrator_->Integrate();
    propagation_time_s_ += propagation_step_s_;
  }

  const libra::Vector<2 * MODE_NUM>& state = integrator_->GetState(); 

  libra::Vector<MODE_NUM> mode_velocities_prev_m_sqrt_kg_s = mode_velocities_m_sqrt_kg_s_;
  for (int i = 0; i < MODE_NUM; i++) {
    mode_displacements_m_sqrt_kg_[i]= state[i];
    mode_velocities_m_sqrt_kg_s_[i] = state[i + MODE_NUM];
  }
  mode_accelerations_m_sqrt_kg_s2_ = (1.0 / compo_delta_time) * (mode_velocities_m_sqrt_kg_s_ - mode_velocities_prev_m_sqrt_kg_s);

  force_to_body_b_N_ = libra::Vector<3>(0.0);
  torque_to_body_b_Nm_ = libra::Vector<3>(0.0);
  for (int i = 0; i < MODE_NUM; i++) {
    force_to_body_b_N_ -= mode_accelerations_m_sqrt_kg_s2_[i] * modes_[i].translation_participation_sqrt_kg;
    torque_to_body_b_Nm_ -= mode_accelerations_m_sqrt_kg_s2_[i] * modes_[i].rotation_participation_sqrt_kg_rad_m;
  }
}

libra::Vector<2 * MODE_NUM> FlexibleStructure::ModeOde::DerivativeFunction(const double independent_variable,
  const libra::Vector<2 * MODE_NUM>& state) const {
  UNUSED(independent_variable);

  libra::Vector<MODE_NUM> x;
  libra::Vector<MODE_NUM> v;
  for (int i = 0; i < MODE_NUM; i++) {
    x[i] = state[i];
    v[i] = state[i + MODE_NUM];
  }

  libra::Vector<MODE_NUM> b = f_ - C_ * v - K_ * x;

  libra::Vector<MODE_NUM> a = SolveLinearSystemWithLu(M_Lu_, index_, b);

  libra::Vector<2 * MODE_NUM> derivative;
  for (int i = 0; i < MODE_NUM; i++) {
    derivative[i] = v[i];
    derivative[i + MODE_NUM] = a[i];
  }
  return derivative;
}

void FlexibleStructure::ModeOde::SetCoeff(const libra::Matrix<MODE_NUM, MODE_NUM>& M, const libra::Vector<MODE_NUM>& c, const libra::Vector<MODE_NUM>& k,
  const libra::Vector<MODE_NUM>& f) {
  libra::Matrix<MODE_NUM, MODE_NUM> M_Lu = M;
  LuDecomposition(M_Lu, index_);

  for (int i = 0; i < MODE_NUM; i++) {
    C_[i][i] = c[i];
    K_[i][i] = k[i];
  }

  f_ = f;
}


inline double FlexibleStructure::GetEnergy() const {
  return InnerProduct(mode_velocities_m_sqrt_kg_s_, mode_velocities_m_sqrt_kg_s_);
}
