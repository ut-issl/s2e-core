/*
 * @file reaction_wheel.hpp
 * @brief Class to emulate Reaction Wheel
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_HPP_
#define S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_HPP_

#include <library/logger/loggable.hpp>
#include <library/logger/logger.hpp>
#include <library/math/vector.hpp>
#include <limits>
#include <string>
#include <vector>

#include "../../base/component.hpp"
#include "reaction_wheel_jitter.hpp"
#include "reaction_wheel_ode.hpp"

/*
 * @class RWModel
 * @brief Class to emulate Reaction Wheel
 * @note For one reaction wheel
 */
class RWModel : public Component, public ILoggable {
 public:
  /**
   * @fn RWModel
   * @brief Constructor without power port
   * @note TODO: argument is too long
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] fast_prescaler: Frequency scale factor for fast update
   * @param [in] clock_generator: Clock generator
   * @param [in] id: Component ID
   * @param [in] step_width: Step width of integration by reaction wheel ordinary differential equation [sec]
   * @param [in] dt_main_routine: Period of execution of main routine of RW [sec]
   * @param [in] jitter_update_interval: Update period of RW jitter [sec]
   * @param [in] inertia: Moment of inertia of the RW [kgm2]
   * @param [in] max_torque: Maximum output torque [Nm]
   * @param [in] max_velocity_rpm: Maximum output angular velocity [RPM]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] pos_b: Position of RW on the body fixed frame [m]
   * @param [in] dead_time: Dead time of torque output [sec]
   * @param [in] driving_lag_coef: Driving lag coefficients
   * @param [in] coasting_lag_coef: Coasting lag coefficients
   * @param [in] is_calc_jitter_enabled: Enable flag to calculate RW jitter
   * @param [in] is_log_jitter_enabled: Enable flag to log output RW jitter
   * @param [in] radial_force_harmonics_coef: Coefficients for radial force harmonics
   * @param [in] radial_torque_harmonics_coef: Coefficients for radial torque harmonics
   * @param [in] structural_resonance_freq: Frequency of structural resonance [Hz]
   * @param [in] damping_factor: Damping factor of structural resonance
   * @param [in] bandwidth: Bandwidth of structural resonance
   * @param [in] considers_structural_resonance: Flag to consider structural resonance
   * @param [in] drive_flag: RW drive flag
   * @param [in] init_velocity: Initial value of angular velocity of RW
   */
  RWModel(const int prescaler, const int fast_prescaler, ClockGenerator *clock_generator, const int id, const double step_width,
          const double dt_main_routine, const double jitter_update_interval, const double inertia, const double max_torque,
          const double max_velocity_rpm, const libra::Quaternion quaternion_b2c, const libra::Vector<3> pos_b, const double dead_time,
          const libra::Vector<3> driving_lag_coef, const libra::Vector<3> coasting_lag_coef, bool is_calc_jitter_enabled, bool is_log_jitter_enabled,
          std::vector<std::vector<double>> radial_force_harmonics_coef, std::vector<std::vector<double>> radial_torque_harmonics_coef,
          double structural_resonance_freq, double damping_factor, double bandwidth, bool considers_structural_resonance,
          const bool drive_flag = false, const double init_velocity = 0.0);
  /**
   * @fn RWModel
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] fast_prescaler: Frequency scale factor for fast update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] id: Component ID
   * @param [in] step_width: Step width of integration by reaction wheel ordinary differential equation [sec]
   * @param [in] dt_main_routine: Period of execution of main routine of RW [sec]
   * @param [in] jitter_update_interval: Update period of RW jitter [sec]
   * @param [in] inertia: Moment of inertia of the RW [kgm2]
   * @param [in] max_torque: Maximum output torque [Nm]
   * @param [in] max_velocity_rpm: Maximum output angular velocity [RPM]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] pos_b: Position of RW on the body fixed frame [m]
   * @param [in] dead_time: Dead time of torque output [sec]
   * @param [in] driving_lag_coef: Driving lag coefficients
   * @param [in] coasting_lag_coef: Coasting lag coefficients
   * @param [in] is_calc_jitter_enabled: Enable flag to calculate RW jitter
   * @param [in] is_log_jitter_enabled: Enable flag to log output RW jitter
   * @param [in] radial_force_harmonics_coef: Coefficients for radial force harmonics
   * @param [in] radial_torque_harmonics_coef: Coefficients for radial torque harmonics
   * @param [in] structural_resonance_freq: Frequency of structural resonance [Hz]
   * @param [in] damping_factor: Damping factor of structural resonance
   * @param [in] bandwidth: Bandwidth of structural resonance
   * @param [in] considers_structural_resonance: Flag to consider structural resonance
   * @param [in] drive_flag: RW drive flag
   * @param [in] init_velocity: Initial value of angular velocity of RW [rad/s]
   */
  RWModel(const int prescaler, const int fast_prescaler, ClockGenerator *clock_generator, PowerPort *power_port, const int id,
          const double step_width, const double dt_main_routine, const double jitter_update_interval, const double inertia, const double max_torque,
          const double max_velocity_rpm, const libra::Quaternion quaternion_b2c, const libra::Vector<3> pos_b, const double dead_time,
          const libra::Vector<3> driving_lag_coef, const libra::Vector<3> coasting_lag_coef, bool is_calc_jitter_enabled, bool is_log_jitter_enabled,
          std::vector<std::vector<double>> radial_force_harmonics_coef, std::vector<std::vector<double>> radial_torque_harmonics_coef,
          double structural_resonance_freq, double damping_factor, double bandwidth, bool considers_structural_resonance,
          const bool drive_flag = false, const double init_velocity = 0.0);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to output torque of normal RW
   */
  void MainRoutine(int count) override;
  /**
   * @fn PowerOffRoutine
   * @brief Power off routine to stop actuation
   */
  void PowerOffRoutine() override;
  /**
   * @fn FastUpdate
   * @brief Main routine to output torque of RW jitter
   */
  void FastUpdate() override;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

  // Getter
  /**
   * @fn GetOutputTorqueB
   * @brief Return output torque in the body fixed frame [Nm]
   */
  const libra::Vector<3> GetOutputTorqueB() const;
  /**
   * @fn GetJitterForceB
   * @brief Return output force by jitter in the body fixed frame [N]
   */
  const libra::Vector<3> GetJitterForceB() const { return rw_jitter_.GetJitterForceB(); }
  /**
   * @fn isMotorDrove
   * @brief Return drive flag
   */
  inline bool isMotorDrove() const { return drive_flag_; };
  /**
   * @fn GetVelocityRad
   * @brief Return angular velocity of RW rotor [rad/s]
   */
  inline double GetVelocityRad() const { return angular_velocity_rad_; };
  /**
   * @fn GetVelocityRpm
   * @brief Return angular velocity of RW rotor [RPM]
   */
  inline double GetVelocityRpm() const { return angular_velocity_rpm_; };
  /**
   * @fn GetAngMomB
   * @brief Return angular momentum of RW [Nms]
   */
  inline const libra::Vector<3> GetAngMomB() const { return angular_momentum_b_; };

  // Setter
  /**
   * @fn SetTargetTorqueRw
   * @brief Set target torque on RW frame [Nm]
   */
  void SetTargetTorqueRw(double torque_rw);
  /**
   * @fn SetTargetTorqueBody
   * @brief Set target torque on body frame (opposite of RW frame) [Nm]
   */
  void SetTargetTorqueBody(double torque_body);
  /**
   * @fn SetVelocityLimitRpm
   * @brief Set velocity limit [RPM]
   */
  void SetVelocityLimitRpm(double velocity_limit_rpm);
  /**
   * @fn SetDriveFlag
   * @brief Set drive flag
   */
  inline void SetDriveFlag(bool flag) { drive_flag_ = flag; };

 protected:
  // Fixed Parameters
  const int id_;                      //!< Actuator ID
  const double inertia_;              //!< Inertia of RW rotor [kgm2]
  const double max_torque_;           //!< Maximum output torque [Nm]
  const double max_velocity_rpm_;     //!< Maximum angular velocity of rotor [rpm]
  libra::Quaternion quaternion_b2c_;  //!< Quaternion from body frame to component frame
  const libra::Vector<3> pos_b_;      //!< Position of RW in the body fixed frame [m]
  libra::Vector<3> direction_c_;      //!< Wheel rotation axis on the component frame. Constant as (0 0 1). (Output torque is minus direction)
  libra::Vector<3> direction_b_;      //!< Wheel rotation vector in the body fixed frame.

  // Fixed Parameters for control delay
  const double step_width_;                   //!< step width for RwOde [sec]
  const double dead_time_;                    //!< dead time [sec]
  const libra::Vector<3> driving_lag_coef_;   //!< delay coefficient for normal drive
  const libra::Vector<3> coasting_lag_coef_;  //!< delay coefficient for coasting drive(Power off)

  // Controlled Parameters
  bool drive_flag_;            //!< Drive flag(True: Drive, False: Stop)
  double velocity_limit_rpm_;  //!< Velocity limit defined by users [RPM]
  double target_accl_;         //!< Target acceleration [rad/s2]

  // Internal variables for control delay
  std::vector<double> delay_buffer_accl_;  //!< Delay buffer for acceleration
  double dt_main_routine_;                 //!< Period of execution of main routine [sec]

  // Output at RW frame
  double angular_acceleration_ = 0.0;  //!< Output angular acceleration [rad/s2]
  double angular_velocity_rpm_ = 0.0;  //!< Current angular velocity [rpm]
  double angular_velocity_rad_ = 0.0;  //!< Current angular velocity [rad/s]

  // Output at body frame
  libra::Vector<3> output_torque_b_{0.0};     //!< Output torque in the body fixed frame [Nm]
  libra::Vector<3> angular_momentum_b_{0.0};  //!< Angular momentum of RW [Nms]

  RwOde ode_angular_velocity_;         //!< Reaction Wheel OrdinaryDifferentialEquation
  RWJitter rw_jitter_;                 //!< RW jitter
  bool is_calculated_jitter_ = false;  //!< Flag for calculation of jitter
  bool is_logged_jitter_ = false;      //!< Flag for log output of jitter

  // Local functions
  /**
   * @fn CalcTorque
   * @brief Calculation of generated torque
   */
  libra::Vector<3> CalcTorque();
  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize();
};

#endif  // S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_HPP_
