/**
 * @file relative_orbit.cpp
 * @brief Class to propagate relative orbit
 */
#include "relative_orbit.hpp"

#include <library/utilities/macros.hpp>

#include "rk4_orbit_propagation.hpp"

RelativeOrbit::RelativeOrbit(const CelestialInformation* celestial_information, double mu, double timestep, int reference_sat_id,
                             Vector<3> initial_relative_position_lvlh, Vector<3> initial_relative_velocity_lvlh,
                             RelativeOrbitUpdateMethod update_method, RelativeOrbitModel relative_dynamics_model_type, STMModel stm_model_type,
                             RelativeInformation* rel_info)
    : Orbit(celestial_information),
      libra::ODE<6>(timestep),
      mu_(mu),
      reference_sat_id_(reference_sat_id),
      update_method_(update_method),
      relative_dynamics_model_type_(relative_dynamics_model_type),
      stm_model_type_(stm_model_type),
      rel_info_(rel_info) {
  propagate_mode_ = OrbitPropagateMode::kRelativeOrbit;

  prop_time_ = 0.0;
  prop_step_ = timestep;

  InitializeState(initial_relative_position_lvlh, initial_relative_velocity_lvlh, mu);
}

RelativeOrbit::~RelativeOrbit() {}

void RelativeOrbit::InitializeState(Vector<3> initial_relative_position_lvlh, Vector<3> initial_relative_velocity_lvlh, double mu, double init_time) {
  relative_position_lvlh_ = initial_relative_position_lvlh;
  relative_velocity_lvlh_ = initial_relative_velocity_lvlh;

  // Disturbance acceleration are not considered in relative orbit propagation
  acc_i_ *= 0;

  Vector<3> reference_sat_position_i = rel_info_->GetReferenceSatDynamics(reference_sat_id_)->GetOrbit().GetSatPosition_i();
  Vector<3> reference_sat_velocity_i = rel_info_->GetReferenceSatDynamics(reference_sat_id_)->GetOrbit().GetSatVelocity_i();
  Quaternion q_i2lvlh = rel_info_->GetReferenceSatDynamics(reference_sat_id_)->GetOrbit().CalcQuaternionI2LVLH();
  Quaternion q_lvlh2i = q_i2lvlh.conjugate();
  sat_position_i_ = q_lvlh2i.frame_conv(relative_position_lvlh_) + reference_sat_position_i;
  sat_velocity_i_ = q_lvlh2i.frame_conv(relative_velocity_lvlh_) + reference_sat_velocity_i;

  initial_state_[0] = initial_relative_position_lvlh[0];
  initial_state_[1] = initial_relative_position_lvlh[1];
  initial_state_[2] = initial_relative_position_lvlh[2];
  initial_state_[3] = initial_relative_velocity_lvlh[0];
  initial_state_[4] = initial_relative_velocity_lvlh[1];
  initial_state_[5] = initial_relative_velocity_lvlh[2];

  if (update_method_ == RK4) {
    setup(init_time, initial_state_);
    CalculateSystemMatrix(relative_dynamics_model_type_, &(rel_info_->GetReferenceSatDynamics(reference_sat_id_)->GetOrbit()), mu);
  } else  // update_method_ == STM
  {
    CalculateSTM(stm_model_type_, &(rel_info_->GetReferenceSatDynamics(reference_sat_id_)->GetOrbit()), mu, 0.0);
  }

  TransEciToEcef();
  TransEcefToGeo();
}

void RelativeOrbit::CalculateSystemMatrix(RelativeOrbitModel relative_dynamics_model_type, const Orbit* reference_sat_orbit, double mu) {
  switch (relative_dynamics_model_type) {
    case RelativeOrbitModel::Hill: {
      double reference_sat_orbit_radius = libra::norm(reference_sat_orbit->GetSatPosition_i());
      system_matrix_ = CalculateHillSystemMatrix(reference_sat_orbit_radius, mu);
    }
    default: {
      // NOT REACHED
      break;
    }
  }
}

void RelativeOrbit::CalculateSTM(STMModel stm_model_type, const Orbit* reference_sat_orbit, double mu, double elapsed_sec) {
  switch (stm_model_type) {
    case STMModel::HCW: {
      double reference_sat_orbit_radius = libra::norm(reference_sat_orbit->GetSatPosition_i());
      stm_ = CalculateHCWSTM(reference_sat_orbit_radius, mu, elapsed_sec);
    }
    default: {
      // NOT REACHED
      break;
    }
  }
}

void RelativeOrbit::Propagate(double endtime, double current_jd) {
  UNUSED(current_jd);

  if (!is_calc_enabled_) return;

  acc_i_ *= 0;  // Disturbance acceleration are not considered in relative orbit propagation

  if (update_method_ == RK4) {
    PropagateRK4(endtime);
  } else  // update_method_ == STM
  {
    PropagateSTM(endtime);
  }

  Vector<3> reference_sat_position_i = rel_info_->GetReferenceSatDynamics(reference_sat_id_)->GetOrbit().GetSatPosition_i();
  Vector<3> reference_sat_velocity_i = rel_info_->GetReferenceSatDynamics(reference_sat_id_)->GetOrbit().GetSatVelocity_i();
  Quaternion q_i2lvlh = rel_info_->GetReferenceSatDynamics(reference_sat_id_)->GetOrbit().CalcQuaternionI2LVLH();
  Quaternion q_lvlh2i = q_i2lvlh.conjugate();

  sat_position_i_ = q_lvlh2i.frame_conv(relative_position_lvlh_) + reference_sat_position_i;
  sat_velocity_i_ = q_lvlh2i.frame_conv(relative_velocity_lvlh_) + reference_sat_velocity_i;
  TransEciToEcef();
  TransEcefToGeo();
}

void RelativeOrbit::PropagateRK4(double elapsed_sec) {
  setStepWidth(prop_step_);  // Re-set propagation dt
  while (elapsed_sec - prop_time_ - prop_step_ > 1.0e-6) {
    Update();  // Propagation methods of the ODE class
    prop_time_ += prop_step_;
  }
  setStepWidth(elapsed_sec - prop_time_);  // Adjust the last propagation dt
  Update();
  prop_time_ = elapsed_sec;

  relative_position_lvlh_[0] = state()[0];
  relative_position_lvlh_[1] = state()[1];
  relative_position_lvlh_[2] = state()[2];
  relative_velocity_lvlh_[0] = state()[3];
  relative_velocity_lvlh_[1] = state()[4];
  relative_velocity_lvlh_[2] = state()[5];
}

void RelativeOrbit::PropagateSTM(double elapsed_sec) {
  Vector<6> current_state;
  CalculateSTM(stm_model_type_, &(rel_info_->GetReferenceSatDynamics(reference_sat_id_)->GetOrbit()), mu_, elapsed_sec);
  current_state = stm_ * initial_state_;

  relative_position_lvlh_[0] = current_state[0];
  relative_position_lvlh_[1] = current_state[1];
  relative_position_lvlh_[2] = current_state[2];
  relative_velocity_lvlh_[0] = current_state[3];
  relative_velocity_lvlh_[1] = current_state[4];
  relative_velocity_lvlh_[2] = current_state[5];
}

void RelativeOrbit::RHS(double t, const Vector<6>& state, Vector<6>& rhs)  // only for RK4 relative dynamics propagation
{
  rhs = system_matrix_ * state;
  (void)t;
}
