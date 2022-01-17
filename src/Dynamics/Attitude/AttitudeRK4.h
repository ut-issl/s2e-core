#ifndef __attitude_rk4_H__
#define __attitude_rk4_H__

#include "Attitude.h"

class AttitudeRK4 : public Attitude, public SimulationObject {
public:
  AttitudeRK4(const Vector<3> &omega_b_ini,
              const Quaternion &quaternion_i2b_ini,
              const Matrix<3, 3> &InertiaTensor_ini,
              const Vector<3> &torque_b_ini, const double prop_step_ini);

  AttitudeRK4(const Vector<3> &omega_b_ini,
              const Quaternion &quaternion_i2b_ini,
              const Matrix<3, 3> &InertiaTensor_ini,
              const Vector<3> &torque_b_ini, const double prop_step_ini,
              std::string name);
  ~AttitudeRK4();

  // MonteCalro
  void SetParameters(const MCSimExecutor &mc_sim);

  virtual void Propagate(double endtime); // 姿勢・角速度のプロパゲーション

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  //デバッグ出力
  void PrintParams(void);

private:
  Vector<3> debug_vec_;

  Matrix<4, 4> Omega4Kinematics(Vector<3> omega);
  Vector<7> DynamicsKinematics(Vector<7> x, double t);
  void RungeOneStep(double t, double dt);
  void CalcAngMom(void);
  void CalcSatRotationalKineticEnergy(void); // 衛星の回転運動エネルギー計算関数
};
#endif //__attitude_rk4_H__
