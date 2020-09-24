#ifndef __RWModel_H__
#define __RWModel_H__
#include "../../Library/math/Vector.hpp"
#include "../../Interface/LogOutput/Logger.h"
#include "../../Interface/LogOutput/ILoggable.h"
#include "../Abstract/ComponentBase.h"
#include "rw_ode.hpp"
#include <string>
#include <vector>
#include <limits>

/**
* @brief リアクションホイールのクラス
* @details 一個分に相当する.コンポジションとしてrw_odeを保持.
*/
class RWModel : public ComponentBase, public ILoggable
{
public:
  /**
  * @brief コンストラクタ
  * @param[in] prescaler and clock_gen : argument for ComponentBase:
  * @param[in] step width: step_width of integration by RwOde [sec]
  * @param[in] dt_main_routine: period of execution of main routine [sec]
  * @param[in] inertia: Moment of inertia of the RW   [kgm2]
  * @param[in] max_torque: Maximum output torque      [Nm]
  * @param[in] max_velocity: Maximum output angular velocity  [RPM]
  * @param[in] direction_b: Mounting direction of the Wheel at Body frame
  * @param[in] dead_time: dead time [sec]
  * @param[in] driving_lag_coef
  * @param[in] coasting_lag_coef
  */
  RWModel(
      int prescaler,
      ClockGenerator *clock_gen,
      double step_width,
      double dt_main_routine,
      double inertia,
      double max_torque,
      double max_velocity_rpm,
      Vector<3> direction_b,
      double dead_time,
      Vector<3> driving_lag_coef,
      Vector<3> coasting_lag_coef,
      bool drive_flag = false,
      double init_velocity = 0.0);

  //Main Routine
  void MainRoutine(int count);
  Vector<3> CalcTorque(); //!トルク計算、発生

  //Getter
  inline const libra::Vector<3> GetOutputTorqueB() const { return output_torque_b_; };
  inline const bool isMotorDrived() const { return drive_flag_; };
  inline const double GetVelocityRad() const { return angular_velocity_rad_; };
  inline const double GetVelocityRpm() const { return angular_velocity_rpm_; };
  inline const libra::Vector<3> GetAngMomB() const { return angular_momentum_b_; };

  //double GetAngularMomentLimit();
  //Setter
  void SetTargetTorqueRw(double torque_rw);
  void SetTargetTorqueBody(double torque_body);
  void SetVelocityLimitRpm(double velocity_limit_rpm);
  inline void SetDriveFlag(bool flag) { drive_flag_ = flag; }; //!動作フラグの設定
                                                               //For Iloggable
  string GetLogHeader() const;
  string GetLogValue() const;

private:
  // Fixed Parameters
  const double inertia_;                     //kg m2
  const double max_torque_;                  //Nm
  const double max_velocity_rpm_;                //rpm
  const libra::Vector<3> direction_b_;       //Mounting direction at body frame
                                             // Fixed Parameters for control delay ()
  const double step_width_;                  //step width for RwOde [sec]
  const double dead_time_;                   //無駄時間 [sec]
  const libra::Vector<3> driving_lag_coef_;  // delay coefficient for normal drive
  const libra::Vector<3> coasting_lag_coef_; // delay coefficient for coasting drive(Power off)

  // Controlled Parameters
  bool drive_flag_;       //!動作フラグ(1で動作、0で停止)
  double velocity_limit_rpm_; //!velocity limit defined by users rpm
  double target_accl_;

  // Internal variables for control delay
  std::vector<double> delay_buffer_accl_; //delay buffer
  double dt_main_routine_;                //period of execution of main routine [sec]

  // Output at RW frame
  double angular_acceleration_; //rad/s2
  double angular_velocity_rpm_; //rpm
  double angular_velocity_rad_; //rad/s

  //Output at body frame
  libra::Vector<3> output_torque_b_;    //Nm
  libra::Vector<3> angular_momentum_b_; //Nms

  RwOde ode_angular_velocity_; //微分方程式のメンバ
};
#endif //__RWModel_H__
