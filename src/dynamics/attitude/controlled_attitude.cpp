/**
 * @file controlled_attitude.hpp
 * @brief Class to calculate spacecraft attitude with Controlled Attitude mode
 */
#include "controlled_attitude.hpp"

#include <logger/log_utility.hpp>
#include <utilities/macros.hpp>

ControlledAttitude::ControlledAttitude(const AttitudeControlMode main_mode, const AttitudeControlMode sub_mode,
                                       const libra::Quaternion quaternion_i2b, const libra::Vector<3> main_target_direction_b,
                                       const libra::Vector<3> sub_target_direction_b, const libra::Matrix<3, 3>& inertia_tensor_kgm2,
                                       const LocalCelestialInformation* local_celestial_information, const Orbit* orbit,
                                       const std::string& simulation_object_name)
    : Attitude(inertia_tensor_kgm2, simulation_object_name),
      main_mode_(main_mode),
      sub_mode_(sub_mode),
      main_target_direction_b_(main_target_direction_b),
      sub_target_direction_b_(sub_target_direction_b),
      local_celestial_information_(local_celestial_information),
      orbit_(orbit) {
  quaternion_i2b_ = quaternion_i2b;

  Initialize();
}

ControlledAttitude::~ControlledAttitude() {}

// Main function
void ControlledAttitude::Initialize(void) {
  if (main_mode_ >= AttitudeControlMode::kNoControl) is_calc_enabled_ = false;
  if (sub_mode_ >= AttitudeControlMode::kNoControl) is_calc_enabled_ = false;
  if (main_mode_ == AttitudeControlMode::kInertialStabilize) {
  } else  // Pointing control
  {
    // sub mode check
    if (main_mode_ == sub_mode_) {
      std::cout << "sub mode should not equal to main mode. \n";
      is_calc_enabled_ = false;
      return;
    }
    // pointing direction check
    main_target_direction_b_ = main_target_direction_b_.CalcNormalizedVector();
    sub_target_direction_b_ = sub_target_direction_b_.CalcNormalizedVector();
    double tmp = InnerProduct(main_target_direction_b_, sub_target_direction_b_);
    tmp = std::abs(tmp);
    if (tmp > cos(kMinDirectionAngle_rad)) {
      std::cout << "sub target direction should separate from the main target direction. \n";
      is_calc_enabled_ = false;
      return;
    }
  }
  return;
}

void ControlledAttitude::Propagate(const double end_time_s) {
  libra::Vector<3> main_direction_i, sub_direction_i;
  if (!is_calc_enabled_) return;

  if (main_mode_ == AttitudeControlMode::kInertialStabilize) {
    // quaternion_i2b_ = quaternion_i2t_;
    return;
  }

  // Calc main target direction
  main_direction_i = CalcTargetDirection_i(main_mode_);
  // Calc sub target direction
  sub_direction_i = CalcTargetDirection_i(sub_mode_);
  // Calc attitude
  PointingControl(main_direction_i, sub_direction_i);
  // Calc angular velocity
  CalcAngularVelocity(end_time_s);
  return;
}

libra::Vector<3> ControlledAttitude::CalcTargetDirection_i(AttitudeControlMode mode) {
  libra::Vector<3> direction;
  if (mode == AttitudeControlMode::kSunPointing) {
    direction = local_celestial_information_->GetPositionFromSpacecraft_i_m("SUN");
    // When the local_celestial_information is not initialized. FIXME: This is temporary codes for attitude initialize.
    if (direction.CalcNorm() == 0.0) {
      libra::Vector<3> sun_position_i_m = local_celestial_information_->GetGlobalInformation().GetPositionFromCenter_i_m("SUN");
      libra::Vector<3> spacecraft_position_i_m = orbit_->GetPosition_i_m();
      direction = sun_position_i_m - spacecraft_position_i_m;
    }
  } else if (mode == AttitudeControlMode::kEarthCenterPointing) {
    direction = local_celestial_information_->GetPositionFromSpacecraft_i_m("EARTH");
    // When the local_celestial_information is not initialized. FIXME: This is temporary codes for attitude initialize.
    if (direction.CalcNorm() == 0.0) {
      libra::Vector<3> earth_position_i_m = local_celestial_information_->GetGlobalInformation().GetPositionFromCenter_i_m("EARTH");
      libra::Vector<3> spacecraft_position_i_m = orbit_->GetPosition_i_m();
      direction = earth_position_i_m - spacecraft_position_i_m;
    }
  } else if (mode == AttitudeControlMode::kVelocityDirectionPointing) {
    direction = orbit_->GetVelocity_i_m_s();
  } else if (mode == AttitudeControlMode::kOrbitNormalPointing) {
    direction = OuterProduct(orbit_->GetPosition_i_m(), orbit_->GetVelocity_i_m_s());
  }
  direction = direction.CalcNormalizedVector();
  return direction;
}

void ControlledAttitude::PointingControl(const libra::Vector<3> main_direction_i, const libra::Vector<3> sub_direction_i) {
  // Calc DCM ECI->Target
  libra::Matrix<3, 3> dcm_t2i = CalcDcm(main_direction_i, sub_direction_i);
  // Calc DCM Target->body
  libra::Matrix<3, 3> dcm_t2b = CalcDcm(main_target_direction_b_, sub_target_direction_b_);
  // Calc DCM ECI->body
  libra::Matrix<3, 3> dcm_i2b = dcm_t2b * dcm_t2i.Transpose();
  // Convert to Quaternion
  quaternion_i2b_ = libra::Quaternion::ConvertFromDcm(dcm_i2b);
}

libra::Matrix<3, 3> ControlledAttitude::CalcDcm(const libra::Vector<3> main_direction, const libra::Vector<3> sub_direction) {
  // Calc basis vectors
  libra::Vector<3> ex, ey, ez;
  ex = main_direction;
  libra::Vector<3> tmp1 = OuterProduct(ex, sub_direction);
  libra::Vector<3> tmp2 = OuterProduct(tmp1, ex);
  ey = tmp2.CalcNormalizedVector();
  libra::Vector<3> tmp3 = OuterProduct(ex, ey);
  ez = tmp3.CalcNormalizedVector();

  // Generate DCM
  libra::Matrix<3, 3> dcm;
  for (int i = 0; i < 3; i++) {
    dcm[i][0] = ex[i];
    dcm[i][1] = ey[i];
    dcm[i][2] = ez[i];
  }
  return dcm;
}

AttitudeControlMode ConvertStringToCtrlMode(const std::string mode) {
  if (mode == "INERTIAL_STABILIZE") {
    return AttitudeControlMode::kInertialStabilize;
  } else if (mode == "SUN_POINTING") {
    return AttitudeControlMode::kSunPointing;
  } else if (mode == "EARTH_CENTER_POINTING") {
    return AttitudeControlMode::kEarthCenterPointing;
  } else if (mode == "VELOCITY_DIRECTION_POINTING") {
    return AttitudeControlMode::kVelocityDirectionPointing;
  } else if (mode == "ORBIT_NORMAL_POINTING") {
    return AttitudeControlMode::kOrbitNormalPointing;
  } else {
    return AttitudeControlMode::kNoControl;
  }
}

void ControlledAttitude::CalcAngularVelocity(const double current_time_s) {
  libra::Vector<3> controlled_torque_b_Nm(0.0);

  if (previous_calc_time_s_ > 0.0) {
    double time_diff_sec = current_time_s - previous_calc_time_s_;
    libra::Quaternion prev_q_b2i = previous_quaternion_i2b_.Conjugate();
    libra::Quaternion q_diff = prev_q_b2i * quaternion_i2b_;
    q_diff = (2.0 / time_diff_sec) * q_diff;

    libra::Vector<3> angular_acc_b_rad_s2_;
    for (int i = 0; i < 3; i++) {
      angular_velocity_b_rad_s_[i] = q_diff[i];
      angular_acc_b_rad_s2_[i] = (previous_omega_b_rad_s_[i] - angular_velocity_b_rad_s_[i]) / time_diff_sec;
    }
    libra::Matrix<3, 3> inv_inertia_tensor = CalcInverseMatrix(inertia_tensor_kgm2_);
    controlled_torque_b_Nm = inv_inertia_tensor * angular_acc_b_rad_s2_;
  } else {
    angular_velocity_b_rad_s_ = libra::Vector<3>(0.0);
    controlled_torque_b_Nm = libra::Vector<3>(0.0);
  }
  // Add torque with disturbances
  AddTorque_b_Nm(controlled_torque_b_Nm);
  // save previous values
  previous_calc_time_s_ = current_time_s;
  previous_quaternion_i2b_ = quaternion_i2b_;
  previous_omega_b_rad_s_ = angular_velocity_b_rad_s_;
}
