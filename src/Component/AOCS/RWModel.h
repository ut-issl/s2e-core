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
* @brief Class for a Reaction Wheel emulation
* @details For one reaction wheel, and it has rw_ode class
*/
class RWModel : public ComponentBase, public ILoggable
{
public:
  /**
  * @brief Constructor
  * @param[in] prescaler and clock_gen : arguments for ComponentBase:
  * @param[in] step_width: step_width of integration by RwOde [sec]
  * @param[in] dt_main_routine: period of execution of main routine [sec]
  * @param[in] inertia: Moment of inertia of the RW   [kgm2]
  * @param[in] max_torque: Maximum output torque      [Nm]
  * @param[in] max_velocity_rpm: Maximum output angular velocity  [RPM]
  * @param[in] direction_b: Mounting direction of the Wheel at Body frame
  * @param[in] dead_time: dead time [sec]
  * @param[in] driving_lag_coef
  * @param[in] coasting_lag_coef
  */
  RWModel(
    const int prescaler,
    ClockGenerator *clock_gen,
    const double step_width,
    const double dt_main_routine,
    const double inertia,
    const double max_torque,
    const double max_velocity_rpm,
    const Vector<3> direction_b,
    const double dead_time,
    const Vector<3> driving_lag_coef,
    const Vector<3> coasting_lag_coef,
    const bool drive_flag = false,
    const double init_velocity = 0.0
  );
  RWModel(
    const int prescaler,
    ClockGenerator *clock_gen,
    PowerPort* power_port,
    const double step_width,
    const double dt_main_routine,
    const double inertia,
    const double max_torque,
    const double max_velocity_rpm,
    const Vector<3> direction_b,
    const double dead_time,
    const Vector<3> driving_lag_coef,
    const Vector<3> coasting_lag_coef,
    const bool drive_flag = false,
    const double init_velocity = 0.0
  );

  // ComponentBase override function
  void MainRoutine(int count) override;

  // Iloggable override function
  string GetLogHeader() const;
  string GetLogValue() const;

  //Getter
  inline const libra::Vector<3> GetOutputTorqueB() const { return output_torque_b_; };
  inline const bool isMotorDrove() const { return drive_flag_; };
  inline const double GetVelocityRad() const { return angular_velocity_rad_; };
  inline const double GetVelocityRpm() const { return angular_velocity_rpm_; };
  inline const libra::Vector<3> GetAngMomB() const { return angular_momentum_b_; };

  //Setter
  void SetTargetTorqueRw(double torque_rw);
  void SetTargetTorqueBody(double torque_body);
  void SetVelocityLimitRpm(double velocity_limit_rpm);
  inline void SetDriveFlag(bool flag) { drive_flag_ = flag; };

protected:
  // Fixed Parameters
  const double inertia_;                     //kg m2
  const double max_torque_;                  //Nm
  const double max_velocity_rpm_;            //rpm
  const libra::Vector<3> direction_b_;       //Mounting direction at body frame

  // Fixed Parameters for control delay
  const double step_width_;                  //step width for RwOde [sec]
  const double dead_time_;                   //dead time [sec]
  const libra::Vector<3> driving_lag_coef_;  // delay coefficient for normal drive
  const libra::Vector<3> coasting_lag_coef_; // delay coefficient for coasting drive(Power off)

  // Controlled Parameters
  bool drive_flag_;           //!drive flag(1: Drive 、0: Stop)
  double velocity_limit_rpm_; //!velocity limit defined by users rpm
  double target_accl_;

  // Internal variables for control delay
  std::vector<double> delay_buffer_accl_; //delay buffer
  double dt_main_routine_;                //period of execution of main routine [sec]

  // Output at RW frame
  double angular_acceleration_=0.0; //rad/s2
  double angular_velocity_rpm_=0.0; //rpm
  double angular_velocity_rad_=0.0; //rad/s

  //Output at body frame
  libra::Vector<3> output_torque_b_{0.0};    //Nm
  libra::Vector<3> angular_momentum_b_{0.0}; //Nms

  RwOde ode_angular_velocity_;

  // Local functions
  Vector<3> CalcTorque();
  void Initialize();
};
#endif //__RWModel_H__
