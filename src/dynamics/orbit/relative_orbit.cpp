/**
 * @file relative_orbit.cpp
 * @brief Class to propagate relative orbit
 */
#include "relative_orbit.hpp"

#include <utilities/macros.hpp>

#include "rk4_orbit_propagation.hpp"

namespace s2e::dynamics::orbit {

RelativeOrbit::RelativeOrbit(const CelestialInformation* celestial_information, double gravity_constant_m3_s2, double time_step_s,
                             int reference_spacecraft_id, s2e::math::Vector<3> relative_position_lvlh_m, s2e::math::Vector<3> relative_velocity_lvlh_m_s,
                             RelativeOrbitUpdateMethod update_method, s2e::orbit::RelativeOrbitModel relative_dynamics_model_type,
                             s2e::orbit::StmModel stm_model_type, RelativeInformation* relative_information)
    : Orbit(celestial_information),
      s2e::math::OrdinaryDifferentialEquation<6>(time_step_s),
      gravity_constant_m3_s2_(gravity_constant_m3_s2),
      reference_spacecraft_id_(reference_spacecraft_id),
      update_method_(update_method),
      relative_dynamics_model_type_(relative_dynamics_model_type),
      stm_model_type_(stm_model_type),
      relative_information_(relative_information) {
  propagate_mode_ = OrbitPropagateMode::kRelativeOrbit;

  propagation_time_s_ = 0.0;
  propagation_step_s_ = time_step_s;

  InitializeState(relative_position_lvlh_m, relative_velocity_lvlh_m_s, gravity_constant_m3_s2);
}

RelativeOrbit::~RelativeOrbit() {}

void RelativeOrbit::InitializeState(s2e::math::Vector<3> relative_position_lvlh_m, s2e::math::Vector<3> relative_velocity_lvlh_m_s,
                                    double gravity_constant_m3_s2, double initial_time_s) {
  relative_position_lvlh_m_ = relative_position_lvlh_m;
  relative_velocity_lvlh_m_s_ = relative_velocity_lvlh_m_s;

  // Disturbance acceleration are not considered in relative orbit propagation
  spacecraft_acceleration_i_m_s2_ *= 0.0;

  s2e::math::Vector<3> reference_sat_position_i = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().GetPosition_i_m();
  s2e::math::Vector<3> reference_sat_velocity_i = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().GetVelocity_i_m_s();
  s2e::math::Quaternion q_i2lvlh = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().CalcQuaternion_i2lvlh();
  s2e::math::Quaternion q_lvlh2i = q_i2lvlh.Conjugate();
  spacecraft_position_i_m_ = q_lvlh2i.FrameConversion(relative_position_lvlh_m_) + reference_sat_position_i;
  spacecraft_velocity_i_m_s_ = q_lvlh2i.FrameConversion(relative_velocity_lvlh_m_s_) + reference_sat_velocity_i;

  initial_state_[0] = relative_position_lvlh_m[0];
  initial_state_[1] = relative_position_lvlh_m[1];
  initial_state_[2] = relative_position_lvlh_m[2];
  initial_state_[3] = relative_velocity_lvlh_m_s[0];
  initial_state_[4] = relative_velocity_lvlh_m_s[1];
  initial_state_[5] = relative_velocity_lvlh_m_s[2];

  if (update_method_ == kRk4) {
    Setup(initial_time_s, initial_state_);
    CalculateSystemMatrix(relative_dynamics_model_type_, &(relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit()),
                          gravity_constant_m3_s2);
  } else  // update_method_ == STM
  {
    CalculateStm(stm_model_type_, &(relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit()), gravity_constant_m3_s2,
                 0.0);
  }

  TransformEciToEcef();
  TransformEcefToGeodetic();
}

void RelativeOrbit::CalculateSystemMatrix(s2e::orbit::RelativeOrbitModel relative_dynamics_model_type, const Orbit* reference_sat_orbit,
                                          double gravity_constant_m3_s2) {
  switch (relative_dynamics_model_type) {
    case s2e::orbit::RelativeOrbitModel::kHill: {
      double reference_sat_orbit_radius = reference_sat_orbit->GetPosition_i_m().CalcNorm();
      system_matrix_ = s2e::orbit::CalcHillSystemMatrix(reference_sat_orbit_radius, gravity_constant_m3_s2);
    }
    default: {
      // NOT REACHED
      break;
    }
  }
}

void RelativeOrbit::CalculateStm(s2e::orbit::StmModel stm_model_type, const Orbit* reference_sat_orbit, double gravity_constant_m3_s2,
                                 double elapsed_sec) {
  switch (stm_model_type) {
    case s2e::orbit::StmModel::kHcw: {
      double reference_sat_orbit_radius = reference_sat_orbit->GetPosition_i_m().CalcNorm();
      stm_ = s2e::orbit::CalcHcwStm(reference_sat_orbit_radius, gravity_constant_m3_s2, elapsed_sec);
    }
    default: {
      // NOT REACHED
      break;
    }
  }
}

void RelativeOrbit::Propagate(const double end_time_s, const double current_time_jd) {
  UNUSED(current_time_jd);

  if (!is_calc_enabled_) return;

  spacecraft_acceleration_i_m_s2_ *= 0.0;  // Disturbance acceleration are not considered in relative orbit propagation

  if (update_method_ == kRk4) {
    PropagateRk4(end_time_s);
  } else  // update_method_ == STM
  {
    PropagateStm(end_time_s);
  }

  s2e::math::Vector<3> reference_sat_position_i = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().GetPosition_i_m();
  s2e::math::Vector<3> reference_sat_velocity_i = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().GetVelocity_i_m_s();
  s2e::math::Quaternion q_i2lvlh = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().CalcQuaternion_i2lvlh();
  s2e::math::Quaternion q_lvlh2i = q_i2lvlh.Conjugate();

  spacecraft_position_i_m_ = q_lvlh2i.FrameConversion(relative_position_lvlh_m_) + reference_sat_position_i;
  spacecraft_velocity_i_m_s_ = q_lvlh2i.FrameConversion(relative_velocity_lvlh_m_s_) + reference_sat_velocity_i;
  TransformEciToEcef();
  TransformEcefToGeodetic();
}

void RelativeOrbit::PropagateRk4(double elapsed_sec) {
  SetStepWidth(propagation_step_s_);  // Re-set propagation dt
  while (elapsed_sec - propagation_time_s_ - propagation_step_s_ > 1.0e-6) {
    Update();  // Propagation methods of the OrdinaryDifferentialEquation class
    propagation_time_s_ += propagation_step_s_;
  }
  SetStepWidth(elapsed_sec - propagation_time_s_);  // Adjust the last propagation dt
  Update();
  propagation_time_s_ = elapsed_sec;

  relative_position_lvlh_m_[0] = GetState()[0];
  relative_position_lvlh_m_[1] = GetState()[1];
  relative_position_lvlh_m_[2] = GetState()[2];
  relative_velocity_lvlh_m_s_[0] = GetState()[3];
  relative_velocity_lvlh_m_s_[1] = GetState()[4];
  relative_velocity_lvlh_m_s_[2] = GetState()[5];
}

void RelativeOrbit::PropagateStm(double elapsed_sec) {
  s2e::math::Vector<6> current_state;
  CalculateStm(stm_model_type_, &(relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit()), gravity_constant_m3_s2_,
               elapsed_sec);
  current_state = stm_ * initial_state_;

  relative_position_lvlh_m_[0] = current_state[0];
  relative_position_lvlh_m_[1] = current_state[1];
  relative_position_lvlh_m_[2] = current_state[2];
  relative_velocity_lvlh_m_s_[0] = current_state[3];
  relative_velocity_lvlh_m_s_[1] = current_state[4];
  relative_velocity_lvlh_m_s_[2] = current_state[5];
}

void RelativeOrbit::DerivativeFunction(double t, const s2e::math::Vector<6>& state,
                                       s2e::math::Vector<6>& rhs)  // only for RK4 relative dynamics propagation
{
  rhs = system_matrix_ * state;
  (void)t;
}

} // namespace s2e::dynamics::orbit
