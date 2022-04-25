#include "ControlledAttitude.h"

#include <Interface/LogOutput/LogUtility.h>

#include <Library/math/Constant.hpp>
#include <Library/utils/Macros.hpp>

using namespace std;

#define THRESHOLD_CA cos(30.0 / 180.0 * libra::pi)  // fix me

ControlledAttitude::ControlledAttitude(const AttCtrlMode main_mode, const AttCtrlMode sub_mode, const Quaternion quaternion_i2t,
                                       const Vector<3> pointing_t_b, const Vector<3> pointing_sub_t_b,
                                       const LocalCelestialInformation* local_celes_info, const Orbit* orbit)
    : local_celes_info_(local_celes_info), orbit_(orbit) {
  main_mode_ = main_mode;
  sub_mode_ = sub_mode;
  quaternion_i2t_ = quaternion_i2t;
  pointing_t_b_ = pointing_t_b;
  pointing_sub_t_b_ = pointing_sub_t_b;
  Initialize();
}

ControlledAttitude::~ControlledAttitude() {}

// Main function
void ControlledAttitude::Initialize(void) {
  quaternion_i2b_ = Quaternion(0, 0, 0, 1);

  if (main_mode_ >= NO_CTRL) IsCalcEnabled = false;
  if (sub_mode_ >= NO_CTRL) IsCalcEnabled = false;
  if (main_mode_ == INERTIAL_STABILIZE) {
    quaternion_i2b_ = quaternion_i2t_;
    quaternion_i2b_.normalize();
  } else  // Pointing control
  {
    // sub mode check
    if (main_mode_ == sub_mode_) {
      cout << "sub mode should not equal to main mode. \n";
      IsCalcEnabled = false;
      return;
    }
    // pointing direction check
    normalize(pointing_t_b_);
    normalize(pointing_sub_t_b_);
    double tmp = inner_product(pointing_t_b_, pointing_sub_t_b_);
    tmp = std::abs(tmp);
    if (tmp > THRESHOLD_CA) {
      cout << "sub target direction should separate from the main target "
              "direction. \n";
      IsCalcEnabled = false;
      return;
    }
  }
  return;
}
void ControlledAttitude::Propagate(double endtime) {
  UNUSED(endtime);

  Vector<3> main_direction_i, sub_direction_i;
  if (!IsCalcEnabled) return;

  if (main_mode_ == INERTIAL_STABILIZE) {
    quaternion_i2b_ = quaternion_i2t_;
    return;
  }

  // Calc main target direction
  main_direction_i = CalcTargetDirection(main_mode_);
  // Calc sub target direction
  sub_direction_i = CalcTargetDirection(sub_mode_);
  // Calc attitude
  PointingCtrl(main_direction_i, sub_direction_i);
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

string ControlledAttitude::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteVector("omega_t", "b", "rad/s", 3);
  str_tmp += WriteVector("q_t", "i2b", "-", 4);

  return str_tmp;
}

string ControlledAttitude::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(omega_b_);
  str_tmp += WriteQuaternion(quaternion_i2b_);

  return str_tmp;
}
