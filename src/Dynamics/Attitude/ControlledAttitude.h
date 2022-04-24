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
                     const Vector<3> pointing_sub_t_b, const LocalCelestialInformation* local_celes_info, const Orbit* orbit,
                     const std::string& sim_object_name = "Attitude");
  ~ControlledAttitude();

  // Setter
  inline void SetMainMode(const AttCtrlMode main_mode) { main_mode_ = main_mode; }
  inline void SetSubMode(const AttCtrlMode sub_mode) { sub_mode_ = sub_mode; }
  inline void SetQuaternionI2T(const Quaternion quaternion_i2t) { quaternion_i2t_ = quaternion_i2t; }
  inline void SetPointingTb(Vector<3> pointing_t_b) { pointing_t_b_ = pointing_t_b; }
  inline void SetPointingSubTb(Vector<3> pointing_sub_t_b) { pointing_sub_t_b_ = pointing_sub_t_b; }

  // Attitude
  virtual void Propagate(const double endtime_s);

 private:
  AttCtrlMode main_mode_;
  AttCtrlMode sub_mode_;               // for control around pointing direction
  libra::Quaternion quaternion_i2t_;   // ECI->Target
  libra::Vector<3> pointing_t_b_;      // Pointing target on body frame
  libra::Vector<3> pointing_sub_t_b_;  // Pointing sub target on body frame

  // Inputs
  const LocalCelestialInformation* local_celes_info_;
  const Orbit* orbit_;

  // Local functions
  void Initialize(void);
  Vector<3> CalcTargetDirection(AttCtrlMode mode);
  void PointingCtrl(const Vector<3> main_direction_i, const Vector<3> sub_direction_i);
  Matrix<3, 3> CalcDCM(const Vector<3> main_direction, const Vector<3> sub_direction);
};

#endif  //__controlled_attitude_H__
