#ifndef __attitude_rk4_H__
#define __attitude_rk4_H__

#include "Attitude.h"

class AttitudeRK4 : public Attitude {
 public:
  AttitudeRK4(const Vector<3>& omega_b_ini, const Quaternion& quaternion_i2b_ini, const Matrix<3, 3>& InertiaTensor_ini,
              const Vector<3>& torque_b_ini, const double prop_step_ini, const std::string& sim_object_name = "Attitude");
  ~AttitudeRK4();

  // Getter
  inline double GetPropTime() const { return prop_time_s_; }

  // Setter
  inline void SetTime(double set) { prop_time_s_ = set; }

  // Attitude
  virtual void Propagate(const double endtime_s);
  virtual void SetParameters(const MCSimExecutor& mc_sim);

 private:
  double prop_time_s_;  // current time

  Matrix<4, 4> Omega4Kinematics(Vector<3> omega);
  Vector<7> DynamicsKinematics(Vector<7> x, double t);
  void RungeOneStep(double t, double dt);
};

#endif  //__attitude_rk4_H__
