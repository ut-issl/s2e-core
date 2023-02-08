/**
 * @file controlled_attitude.hpp
 * @brief Class to calculate spacecraft attitude with Controlled Attitude mode
 */
#include "controlled_attitude.hpp"

#include <library/math/Constant.hpp>
#include <library/utilities/macros.hpp>
#include <interface/log_output/log_utility.hpp>

using namespace std;

#define THRESHOLD_CA cos(30.0 / 180.0 * libra::pi)  // fix me

ControlledAttitude::ControlledAttitude(const AttCtrlMode main_mode, const AttCtrlMode sub_mode, const Quaternion quaternion_i2b,
                                       const Vector<3> pointing_t_b, const Vector<3> pointing_sub_t_b, const Matrix<3, 3>& inertia_tensor_kgm2,
                                       const LocalCelestialInformation* local_celes_info, const Orbit* orbit, const std::string& sim_object_name)
    : Attitude(sim_object_name),
      main_mode_(main_mode),
      sub_mode_(sub_mode),
      pointing_t_b_(pointing_t_b),
      pointing_sub_t_b_(pointing_sub_t_b),
      local_celes_info_(local_celes_info),
      orbit_(orbit) {
  quaternion_i2b_ = quaternion_i2b;
  inertia_tensor_kgm2_ = inertia_tensor_kgm2;  // FIXME: inertia tensor should be initialized in the Attitude base class
  inv_inertia_tensor_ = invert(inertia_tensor_kgm2_);

  Initialize();
}

ControlledAttitude::~ControlledAttitude() {}

// Main function
void ControlledAttitude::Initialize(void) {
  if (main_mode_ >= NO_CTRL) is_calc_enabled_ = false;
  if (sub_mode_ >= NO_CTRL) is_calc_enabled_ = false;
  if (main_mode_ == INERTIAL_STABILIZE) {
  } else  // Pointing control
  {
    // sub mode check
    if (main_mode_ == sub_mode_) {
      cout << "sub mode should not equal to main mode. \n";
      is_calc_enabled_ = false;
      return;
    }
    // pointing direction check
    normalize(pointing_t_b_);
    normalize(pointing_sub_t_b_);
    double tmp = inner_product(pointing_t_b_, pointing_sub_t_b_);
    tmp = std::abs(tmp);
    if (tmp > THRESHOLD_CA) {
      cout << "sub target direction should separate from the main target direction. \n";
      is_calc_enabled_ = false;
      return;
    }
  }
  return;
}

void ControlledAttitude::Propagate(const double endtime_s) {
  Vector<3> main_direction_i, sub_direction_i;
  if (!is_calc_enabled_) return;

  if (main_mode_ == INERTIAL_STABILIZE) {
    // quaternion_i2b_ = quaternion_i2t_;
    return;
  }

  // Calc main target direction
  main_direction_i = CalcTargetDirection(main_mode_);
  // Calc sub target direction
  sub_direction_i = CalcTargetDirection(sub_mode_);
  // Calc attitude
  PointingCtrl(main_direction_i, sub_direction_i);
  // Calc angular velocity
  CalcAngularVelocity(endtime_s);
  return;
}

Vector<3> ControlledAttitude::CalcTargetDirection(AttCtrlMode mode) {
  Vector<3> direction;
  if (mode == SUN_POINTING) {
    direction = local_celes_info_->GetPosFromSC_i("SUN");
  } else if (mode == EARTH_CENTER_POINTING) {
    direction = local_celes_info_->GetPosFromSC_i("EARTH");
  } else if (mode == VELOCITY_DIRECTION_POINTING) {
    direction = orbit_->GetSatVelocity_i();
  } else if (mode == ORBIT_NORMAL_POINTING) {
    direction = outer_product(orbit_->GetSatPosition_i(), orbit_->GetSatVelocity_i());
  }
  normalize(direction);
  return direction;
}

void ControlledAttitude::PointingCtrl(const Vector<3> main_direction_i, const Vector<3> sub_direction_i) {
  // Calc DCM ECI->Target
  Matrix<3, 3> DCM_t2i = CalcDCM(main_direction_i, sub_direction_i);
  // Calc DCM Target->body
  Matrix<3, 3> DCM_t2b = CalcDCM(pointing_t_b_, pointing_sub_t_b_);
  // Calc DCM ECI->body
  Matrix<3, 3> DCM_i2b = DCM_t2b * transpose(DCM_t2i);
  // Convert to Quaternion
  quaternion_i2b_ = Quaternion::fromDCM(DCM_i2b);
}

Matrix<3, 3> ControlledAttitude::CalcDCM(const Vector<3> main_direction, const Vector<3> sub_direction) {
  // Calc basis vectors
  Vector<3> ex, ey, ez;
  ex = main_direction;
  Vector<3> tmp1 = outer_product(ex, sub_direction);
  Vector<3> tmp2 = outer_product(tmp1, ex);
  ey = normalize(tmp2);
  Vector<3> tmp3 = outer_product(ex, ey);
  ez = normalize(tmp3);

  // Generate DCM
  Matrix<3, 3> DCM_;
  for (int i = 0; i < 3; i++) {
    DCM_[i][0] = ex[i];
    DCM_[i][1] = ey[i];
    DCM_[i][2] = ez[i];
  }
  return DCM_;
}

AttCtrlMode ConvertStringToCtrlMode(const std::string mode) {
  if (mode == "INERTIAL_STABILIZE") {
    return INERTIAL_STABILIZE;
  } else if (mode == "SUN_POINTING") {
    return SUN_POINTING;
  } else if (mode == "EARTH_CENTER_POINTING") {
    return EARTH_CENTER_POINTING;
  } else if (mode == "VELOCITY_DIRECTION_POINTING") {
    return VELOCITY_DIRECTION_POINTING;
  } else if (mode == "ORBIT_NORMAL_POINTING") {
    return ORBIT_NORMAL_POINTING;
  } else {
    return NO_CTRL;
  }
}

void ControlledAttitude::CalcAngularVelocity(const double current_time_s) {
  libra::Vector<3> controlled_torque_b_Nm(0.0);

  if (previous_calc_time_s_ > 0.0) {
    double time_diff_sec = current_time_s - previous_calc_time_s_;
    libra::Quaternion prev_q_b2i = prev_quaternion_i2b_.conjugate();
    libra::Quaternion q_diff = prev_q_b2i * quaternion_i2b_;
    q_diff = (2.0 / time_diff_sec) * q_diff;

    libra::Vector<3> angular_acc_b_rad_s2_;
    for (int i = 0; i < 3; i++) {
      omega_b_rad_s_[i] = q_diff[i];
      angular_acc_b_rad_s2_[i] = (prev_omega_b_rad_s_[i] - omega_b_rad_s_[i]) / time_diff_sec;
    }
    controlled_torque_b_Nm = inv_inertia_tensor_ * angular_acc_b_rad_s2_;
  } else {
    omega_b_rad_s_ = libra::Vector<3>(0.0);
    controlled_torque_b_Nm = libra::Vector<3>(0.0);
  }
  // Add torque with disturbances
  AddTorque_b(controlled_torque_b_Nm);
  // save previous values
  previous_calc_time_s_ = current_time_s;
  prev_quaternion_i2b_ = quaternion_i2b_;
  prev_omega_b_rad_s_ = omega_b_rad_s_;
}
