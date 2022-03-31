#ifndef __RWModel_H__
#define __RWModel_H__
#include <Interface/LogOutput/ILoggable.h>
#include <Interface/LogOutput/Logger.h>

#include <Library/math/Vector.hpp>
#include <limits>
#include <string>
#include <vector>

#include "../Abstract/ComponentBase.h"
#include "RWJitter.h"
#include "rw_ode.hpp"

/**
 * @brief Class for a Reaction Wheel emulation
 * @details For one reaction wheel, and it has rw_ode class
 */
class RWModel : public ComponentBase, public ILoggable {
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
  RWModel(const int prescaler, const int fast_prescaler, ClockGenerator *clock_gen, const double step_width, const double dt_main_routine,
          const double jitter_update_interval, const double inertia, const double max_torque, const double max_velocity_rpm, const Quaternion q_b2c,
          const Vector<3> pos_b, const double dead_time, const Vector<3> driving_lag_coef, const Vector<3> coasting_lag_coef,
          bool is_calc_jitter_enabled, bool is_log_jitter_enabled, std::vector<std::vector<double>> radial_force_harmonics_coef,
          std::vector<std::vector<double>> radial_torque_harmonics_coef, double structural_resonance_freq, double damping_factor, double bandwidth,
          bool considers_structural_resonance, const bool drive_flag = false, const double init_velocity = 0.0);
  RWModel(const int prescaler, const int fast_prescaler, ClockGenerator *clock_gen, PowerPort *power_port, const double step_width,
          const double dt_main_routine, const double jitter_update_interval, const double inertia, const double max_torque,
          const double max_velocity_rpm, const Quaternion q_b2c, const Vector<3> pos_b, const double dead_time, const Vector<3> driving_lag_coef,
          const Vector<3> coasting_lag_coef, bool is_calc_jitter_enabled, bool is_log_jitter_enabled,
          std::vector<std::vector<double>> radial_force_harmonics_coef, std::vector<std::vector<double>> radial_torque_harmonics_coef,
          double structural_resonance_freq, double damping_factor, double bandwidth, bool considers_structural_resonance,
          const bool drive_flag = false, const double init_velocity = 0.0);

  // ComponentBase override function
  void MainRoutine(int count) override;
  // Fast Update for jitter
  void FastUpdate() override;

  // Iloggable override function
  std::string GetLogHeader() const;
  std::string GetLogValue() const;

  // Getter
  const libra::Vector<3> GetOutputTorqueB() const;
  const libra::Vector<3> GetJitterForceB() const { return rw_jitter_.GetJitterForceB(); }
  inline const bool isMotorDrove() const { return drive_flag_; };
  inline const double GetVelocityRad() const { return angular_velocity_rad_; };
  inline const double GetVelocityRpm() const { return angular_velocity_rpm_; };
  inline const libra::Vector<3> GetAngMomB() const { return angular_momentum_b_; };

  // Setter
  void SetTargetTorqueRw(double torque_rw);
  void SetTargetTorqueBody(double torque_body);
  void SetVelocityLimitRpm(double velocity_limit_rpm);
  inline void SetDriveFlag(bool flag) { drive_flag_ = flag; };

 protected:
  // Fixed Parameters
  const double inertia_;           // kg m2
  const double max_torque_;        // Nm
  const double max_velocity_rpm_;  // rpm
  libra::Quaternion q_b2c_;        // Quaternion from body frame to component frame
  const libra::Vector<3> pos_b_;
  libra::Vector<3> direction_c_;  // Definition of component frame : wheel rotation axis = (0
                                  // 0 1)^T. plus means direction of rotation (output torque
                                  // is minus direction)
  libra::Vector<3> direction_b_;  // Wheel rotation vector in body frame. direction_b_ means
                                  // direction of wheel rotation, so output torque direction
                                  // is opposite to direction_b_.

  // Fixed Parameters for control delay
  const double step_width_;                   // step width for RwOde [sec]
  const double dead_time_;                    // dead time [sec]
  const libra::Vector<3> driving_lag_coef_;   // delay coefficient for normal drive
  const libra::Vector<3> coasting_lag_coef_;  // delay coefficient for coasting drive(Power off)

  // Controlled Parameters
  bool drive_flag_;            //! drive flag(1: Drive 、0: Stop)
  double velocity_limit_rpm_;  //! velocity limit defined by users rpm
  double target_accl_;

  // Internal variables for control delay
  std::vector<double> delay_buffer_accl_;  // delay buffer
  double dt_main_routine_;                 // period of execution of main routine [sec]

  // Output at RW frame
  double angular_acceleration_ = 0.0;  // rad/s2
  double angular_velocity_rpm_ = 0.0;  // rpm
  double angular_velocity_rad_ = 0.0;  // rad/s

  // Output at body frame
  libra::Vector<3> output_torque_b_{0.0};     // Nm
  libra::Vector<3> angular_momentum_b_{0.0};  // Nms

  RwOde ode_angular_velocity_;

  // Local functions
  Vector<3> CalcTorque();
  void Initialize();

  // RW jitter model
  RWJitter rw_jitter_;
  bool is_calculated_jitter_ = false;
  bool is_logged_jitter_ = false;
};
#endif  //__RWModel_H__
