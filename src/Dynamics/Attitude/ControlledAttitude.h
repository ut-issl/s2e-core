#ifndef __controlled_attitude_H__
#define __controlled_attitude_H__

#include <Environment/Local/LocalCelestialInformation.h>

#include <string>

#include "../Orbit/Orbit.h"
#include "Attitude.h"

enum AttCtrlMode {
  INERTIAL_STABILIZE,
  SUN_POINTING,
  EARTH_CENTER_POINTING,
  VELOCITY_DIRECTION_POINTING,
  ORBIT_NORMAL_POINTING,
  NO_CTRL,
};

AttCtrlMode ConvertStringToCtrlMode(const std::string mode);

class ControlledAttitude : public Attitude {
 public:
  ControlledAttitude(const AttCtrlMode main_mode, const AttCtrlMode sub_mode, const Quaternion quaternion_i2b, const Vector<3> pointing_t_b,
                     const Vector<3> pointing_sub_t_b, const Matrix<3, 3>& inertia_tensor_kgm2, const LocalCelestialInformation* local_celes_info,
                     const Orbit* orbit, const std::string& sim_object_name = "Attitude");
  ~ControlledAttitude();

  // Setter
  inline void SetMainMode(const AttCtrlMode main_mode) { main_mode_ = main_mode; }
  inline void SetSubMode(const AttCtrlMode sub_mode) { sub_mode_ = sub_mode; }
  inline void SetQuaternionI2T(const Quaternion quaternion_i2t) { quaternion_i2b_ = quaternion_i2t; }
  inline void SetPointingTb(Vector<3> pointing_t_b) { pointing_t_b_ = pointing_t_b; }
  inline void SetPointingSubTb(Vector<3> pointing_sub_t_b) { pointing_sub_t_b_ = pointing_sub_t_b; }

  // Attitude
  virtual void Propagate(const double endtime_s);

 private:
  AttCtrlMode main_mode_;
  AttCtrlMode sub_mode_;                   //!< for control around pointing direction
  libra::Vector<3> pointing_t_b_;          //!< Pointing target on body frame
  libra::Vector<3> pointing_sub_t_b_;      //!< Pointing sub target on body frame
  double previous_calc_time_s_ = -1.0;     //!< previous time of velocity calculation
  libra::Quaternion prev_quaternion_i2b_;  //!< previous quaternion
  libra::Vector<3> prev_omega_b_rad_s_;    //!< previous angular velocity
  
  // Inputs
  const LocalCelestialInformation* local_celes_info_;
  const Orbit* orbit_;

  // Local functions
  void Initialize(void);
  Vector<3> CalcTargetDirection(AttCtrlMode mode);
  void PointingCtrl(const Vector<3> main_direction_i, const Vector<3> sub_direction_i);
  void CalcAngularVelocity(const double current_time_s);
  Matrix<3, 3> CalcDCM(const Vector<3> main_direction, const Vector<3> sub_direction);
};

#endif  //__controlled_attitude_H__
