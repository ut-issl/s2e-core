/**
 * @file relative_orbit.cpp
 * @brief Class to propagate relative orbit
 */
#include "relative_orbit.hpp"

#include <utilities/macros.hpp>

#include "rk4_orbit_propagation.hpp"

RelativeOrbit::RelativeOrbit(const CelestialInformation* celestial_information, double gravity_constant_m3_s2, double time_step_s,
                             int reference_spacecraft_id, math::Vector<3> relative_position_lvlh_m, math::Vector<3> relative_velocity_lvlh_m_s,
                             RelativeOrbitUpdateMethod update_method, orbit::RelativeOrbitModel relative_dynamics_model_type,
                             orbit::StmModel stm_model_type, RelativeInformation* relative_information)
    : Orbit(celestial_information),
      math::OrdinaryDifferentialEquation<6>(time_step_s),
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

void RelativeOrbit::InitializeState(math::Vector<3> relative_position_lvlh_m, math::Vector<3> relative_velocity_lvlh_m_s,
                                    double gravity_constant_m3_s2, double initial_time_s) {
  relative_position_lvlh_m_ = relative_position_lvlh_m;
  relative_velocity_lvlh_m_s_ = relative_velocity_lvlh_m_s;

  // Disturbance acceleration are not considered in relative orbit propagation
  spacecraft_acceleration_i_m_s2_ *= 0.0;

  math::Vector<3> reference_sat_position_i = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().GetPosition_i_m();
  math::Vector<3> reference_sat_velocity_i = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().GetVelocity_i_m_s();
  math::Quaternion q_i2lvlh = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().CalcQuaternion_i2lvlh();
  math::Quaternion q_lvlh2i = q_i2lvlh.Conjugate();
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
    InitializeStmMatrix(stm_model_type_, &(relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit()),
                        gravity_constant_m3_s2, 0.0);
    CalculateStm(stm_model_type_, &(relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit()), gravity_constant_m3_s2,
                 0.0);
  }

  TransformEciToEcef();
  TransformEcefToGeodetic();
}

void RelativeOrbit::CalculateSystemMatrix(orbit::RelativeOrbitModel relative_dynamics_model_type, const Orbit* reference_sat_orbit,
                                          double gravity_constant_m3_s2) {
  switch (relative_dynamics_model_type) {
    case orbit::RelativeOrbitModel::kHill: {
      double reference_sat_orbit_radius = reference_sat_orbit->GetPosition_i_m().CalcNorm();
      system_matrix_ = orbit::CalcHillSystemMatrix(reference_sat_orbit_radius, gravity_constant_m3_s2);
    }
    default: {
      // NOT REACHED
      break;
    }
  }
}

void RelativeOrbit::InitializeStmMatrix(orbit::StmModel stm_model_type, const Orbit* reference_sat_orbit, double gravity_constant_m3_s2,
                                        double elapsed_sec) {
  orbit::OrbitalElements reference_oe =
      orbit::OrbitalElements(gravity_constant_m3_s2_, elapsed_sec, reference_sat_orbit->GetPosition_i_m(), reference_sat_orbit->GetVelocity_i_m_s());
  math::Vector<3> position_i_m = reference_sat_orbit->GetPosition_i_m();
  double reference_sat_orbit_radius = position_i_m.CalcNorm();
  // Temporary codes for the integration by true anomaly
  double raan_rad = reference_oe.GetRaan_rad();
  double inclination_rad = reference_oe.GetInclination_rad();
  double arg_perigee_rad = reference_oe.GetArgPerigee_rad();
  double x_p_m = position_i_m[0] * cos(raan_rad) + position_i_m[1] * sin(raan_rad);
  double tmp_m = -position_i_m[0] * sin(raan_rad) + position_i_m[1] * cos(raan_rad);
  double y_p_m = tmp_m * cos(inclination_rad) + position_i_m[2] * sin(inclination_rad);
  double phi_rad = atan2(y_p_m, x_p_m);
  double f_ref_rad = phi_rad - arg_perigee_rad;

  switch (stm_model_type) {
    case orbit::StmModel::kYamakawaAnkersen:
      relative_orbit_yamanaka_ankersen_.CalculateInitialInverseMatrix(f_ref_rad, &reference_oe);
      break;

    default:
      break;
  }
}

void RelativeOrbit::CalculateStm(orbit::StmModel stm_model_type, const Orbit* reference_sat_orbit, double gravity_constant_m3_s2,
                                 double elapsed_sec) {
  orbit::OrbitalElements reference_oe =
      orbit::OrbitalElements(gravity_constant_m3_s2_, elapsed_sec, reference_sat_orbit->GetPosition_i_m(), reference_sat_orbit->GetVelocity_i_m_s());
  math::Vector<3> position_i_m = reference_sat_orbit->GetPosition_i_m();
  double reference_sat_orbit_radius = position_i_m.CalcNorm();
  // Temporary codes for the integration by true anomaly
  double raan_rad = reference_oe.GetRaan_rad();
  double inclination_rad = reference_oe.GetInclination_rad();
  double arg_perigee_rad = reference_oe.GetArgPerigee_rad();
  double x_p_m = position_i_m[0] * cos(raan_rad) + position_i_m[1] * sin(raan_rad);
  double tmp_m = -position_i_m[0] * sin(raan_rad) + position_i_m[1] * cos(raan_rad);
  double y_p_m = tmp_m * cos(inclination_rad) + position_i_m[2] * sin(inclination_rad);
  double phi_rad = atan2(y_p_m, x_p_m);
  double f_ref_rad = phi_rad - arg_perigee_rad;

  switch (stm_model_type) {
    case orbit::StmModel::kHcw: {
      stm_ = orbit::CalcHcwStm(reference_sat_orbit_radius, gravity_constant_m3_s2, elapsed_sec);
      break;
    }
    case orbit::StmModel::kMelton: {
      stm_ = orbit::CalcMeltonStm(reference_sat_orbit_radius, gravity_constant_m3_s2, elapsed_sec, &reference_oe);
      break;
    }
    case orbit::StmModel::kSs: {
      stm_ = orbit::CalcSsStm(reference_sat_orbit_radius, gravity_constant_m3_s2, elapsed_sec, &reference_oe);
      correction_term_ = orbit::CalcSsCorrectionTerm(reference_sat_orbit_radius, gravity_constant_m3_s2, elapsed_sec, &reference_oe);
      break;
    }
    case orbit::StmModel::kSabatini: {
      stm_ = orbit::CalcSabatiniStm(reference_sat_orbit_radius, gravity_constant_m3_s2, elapsed_sec, &reference_oe);
      break;
    }
    case orbit::StmModel::kCarter: {
      stm_ = orbit::CalcCarterStm(reference_sat_orbit_radius, gravity_constant_m3_s2, f_ref_rad, &reference_oe);
      break;
    }
    case orbit::StmModel::kYamakawaAnkersen: {
      stm_ = relative_orbit_yamanaka_ankersen_.CalculateSTM(gravity_constant_m3_s2, elapsed_sec, f_ref_rad, &reference_oe);
      break;
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

  math::Vector<3> reference_sat_position_i = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().GetPosition_i_m();
  math::Vector<3> reference_sat_velocity_i = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().GetVelocity_i_m_s();
  math::Quaternion q_i2lvlh = relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit().CalcQuaternion_i2lvlh();
  math::Quaternion q_lvlh2i = q_i2lvlh.Conjugate();

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
  math::Vector<6> current_state;
  CalculateStm(stm_model_type_, &(relative_information_->GetReferenceSatDynamics(reference_spacecraft_id_)->GetOrbit()), gravity_constant_m3_s2_,
               elapsed_sec);
  current_state = stm_ * initial_state_ + correction_term_;

  relative_position_lvlh_m_[0] = current_state[0];
  relative_position_lvlh_m_[1] = current_state[1];
  relative_position_lvlh_m_[2] = current_state[2];
  relative_velocity_lvlh_m_s_[0] = current_state[3];
  relative_velocity_lvlh_m_s_[1] = current_state[4];
  relative_velocity_lvlh_m_s_[2] = current_state[5];
}

void RelativeOrbit::DerivativeFunction(double t, const math::Vector<6>& state,
                                       math::Vector<6>& rhs)  // only for RK4 relative dynamics propagation
{
  rhs = system_matrix_ * state;
  (void)t;
}
