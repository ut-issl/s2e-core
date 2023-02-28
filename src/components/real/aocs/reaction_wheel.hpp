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
   * @param [in] component_id: Component ID
   * @param [in] step_width_s: Step width of integration by reaction wheel ordinary differential equation [sec]
   * @param [in] main_routine_time_step_s: Period of execution of main routine of RW [sec]
   * @param [in] jitter_update_interval_s: Update period of RW jitter [sec]
   * @param [in] rotor_inertia_kgm2: Moment of rotor_inertia_kgm2 of the RW [kgm2]
   * @param [in] max_torque_Nm: Maximum output torque [Nm]
   * @param [in] max_velocity_rpm: Maximum output angular velocity [RPM]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] position_b_m: Position of RW on the body fixed frame [m]
   * @param [in] dead_time_s: Dead time of torque output [sec]
   * @param [in] driving_lag_coefficients: Driving lag coefficients
   * @param [in] coasting_lag_coefficients: Coasting lag coefficients
   * @param [in] is_calc_jitter_enabled: Enable flag to calculate RW jitter
   * @param [in] is_log_jitter_enabled: Enable flag to log output RW jitter
   * @param [in] radial_force_harmonics_coefficients: Coefficients for radial force harmonics
   * @param [in] radial_torque_harmonics_coefficients: Coefficients for radial torque harmonics
   * @param [in] structural_resonance_frequency_Hz: Frequency of structural resonance [Hz]
   * @param [in] damping_factor: Damping factor of structural resonance
   * @param [in] bandwidth: Bandwidth of structural resonance
   * @param [in] considers_structural_resonance: Flag to consider structural resonance
   * @param [in] drive_flag: RW drive flag
   * @param [in] init_velocity_rad_s: Initial value of angular velocity of RW
   */
  RWModel(const int prescaler, const int fast_prescaler, ClockGenerator *clock_generator, const int component_id, const double step_width_s,
          const double main_routine_time_step_s, const double jitter_update_interval_s, const double rotor_inertia_kgm2, const double max_torque_Nm,
          const double max_velocity_rpm, const libra::Quaternion quaternion_b2c, const libra::Vector<3> position_b_m, const double dead_time_s,
          const libra::Vector<3> driving_lag_coefficients, const libra::Vector<3> coasting_lag_coefficients, const bool is_calc_jitter_enabled,
          const bool is_log_jitter_enabled, const std::vector<std::vector<double>> radial_force_harmonics_coefficients,
          const std::vector<std::vector<double>> radial_torque_harmonics_coefficients, const double structural_resonance_frequency_Hz,
          const double damping_factor, const double bandwidth, const bool considers_structural_resonance, const bool drive_flag = false,
          const double init_velocity_rad_s = 0.0);
  /**
   * @fn RWModel
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] fast_prescaler: Frequency scale factor for fast update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] component_id: Component ID
   * @param [in] step_width_s: Step width of integration by reaction wheel ordinary differential equation [sec]
   * @param [in] main_routine_time_step_s: Period of execution of main routine of RW [sec]
   * @param [in] jitter_update_interval_s: Update period of RW jitter [sec]
   * @param [in] rotor_inertia_kgm2: Moment of rotor_inertia_kgm2 of the RW [kgm2]
   * @param [in] max_torque_Nm: Maximum output torque [Nm]
   * @param [in] max_velocity_rpm: Maximum output angular velocity [RPM]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] position_b_m: Position of RW on the body fixed frame [m]
   * @param [in] dead_time_s: Dead time of torque output [sec]
   * @param [in] driving_lag_coefficients: Driving lag coefficients
   * @param [in] coasting_lag_coefficients: Coasting lag coefficients
   * @param [in] is_calc_jitter_enabled: Enable flag to calculate RW jitter
   * @param [in] is_log_jitter_enabled: Enable flag to log output RW jitter
   * @param [in] radial_force_harmonics_coefficients: Coefficients for radial force harmonics
   * @param [in] radial_torque_harmonics_coefficients: Coefficients for radial torque harmonics
   * @param [in] structural_resonance_frequency_Hz: Frequency of structural resonance [Hz]
   * @param [in] damping_factor: Damping factor of structural resonance
   * @param [in] bandwidth: Bandwidth of structural resonance
   * @param [in] considers_structural_resonance: Flag to consider structural resonance
   * @param [in] drive_flag: RW drive flag
   * @param [in] init_velocity_rad_s: Initial value of angular velocity of RW [rad/s]
   */
  RWModel(const int prescaler, const int fast_prescaler, ClockGenerator *clock_generator, PowerPort *power_port, const int component_id,
          const double step_width_s, const double main_routine_time_step_s, const double jitter_update_interval_s, const double rotor_inertia_kgm2,
          const double max_torque_Nm, const double max_velocity_rpm, const libra::Quaternion quaternion_b2c, const libra::Vector<3> position_b_m,
          const double dead_time_s, const libra::Vector<3> driving_lag_coefficients, const libra::Vector<3> coasting_lag_coefficients,
          const bool is_calc_jitter_enabled, const bool is_log_jitter_enabled,
          const std::vector<std::vector<double>> radial_force_harmonics_coefficients,
          const std::vector<std::vector<double>> radial_torque_harmonics_coefficients, const double structural_resonance_frequency_Hz,
          const double damping_factor, const double bandwidth, const bool considers_structural_resonance, const bool drive_flag = false,
          const double init_velocity_rad_s = 0.0);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to output torque of normal RW
   */
  void MainRoutine(const int time_count) override;
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
   * @fn GetJitterForce_b_N
   * @brief Return output force by jitter in the body fixed frame [N]
   */
  const libra::Vector<3> GetJitterForce_b_N() const { return rw_jitter_.GetJitterForce_b_N(); }
  /**
   * @fn isMotorDrove
   * @brief Return drive flag
   */
  inline bool isMotorDrove() const { return drive_flag_; };
  /**
   * @fn GetVelocityRad
   * @brief Return angular velocity of RW rotor [rad/s]
   */
  inline double GetVelocityRad() const { return angular_velocity_rad_s_; };
  /**
   * @fn GetVelocityRpm
   * @brief Return angular velocity of RW rotor [RPM]
   */
  inline double GetVelocityRpm() const { return angular_velocity_rpm_; };
  /**
   * @fn GetAngMomB
   * @brief Return angular momentum of RW [Nms]
   */
  inline const libra::Vector<3> GetAngMomB() const { return angular_momentum_b_Nms_; };

  // Setter
  /**
   * @fn SetTargetTorqueRw
   * @brief Set target torque on RW frame [Nm]
   */
  void SetTargetTorqueRw(const double torque_rw_Nm);
  /**
   * @fn SetTargetTorqueBody
   * @brief Set target torque on body frame (opposite of RW frame) [Nm]
   */
  void SetTargetTorqueBody(const double torque_b_Nm);
  /**
   * @fn SetVelocityLimitRpm
   * @brief Set velocity limit [RPM]
   */
  void SetVelocityLimitRpm(const double velocity_limit_rpm);
  /**
   * @fn SetDriveFlag
   * @brief Set drive flag
   */
  inline void SetDriveFlag(const bool flag) { drive_flag_ = flag; };

 protected:
  // Fixed Parameters
  const int component_id_;               //!< Actuator ID
  const double rotor_inertia_kgm2_;      //!< Inertia of RW rotor [kgm2]
  const double max_torque_Nm_;           //!< Maximum output torque [Nm]
  const double max_velocity_rpm_;        //!< Maximum angular velocity of rotor [rpm]
  libra::Quaternion quaternion_b2c_;     //!< Quaternion from body frame to component frame
  const libra::Vector<3> position_b_m_;  //!< Position of RW in the body fixed frame [m]
  libra::Vector<3> rotation_axis_c_;     //!< Wheel rotation axis on the component frame. Constant as (0 0 1). (Output torque is minus direction)
  libra::Vector<3> rotation_axis_b_;     //!< Wheel rotation vector in the body fixed frame.

  // Fixed Parameters for control delay
  const double step_width_s_;                         //!< step width for ReactionWheelOde [sec]
  const double dead_time_s_;                          //!< dead time [sec]
  const libra::Vector<3> driving_lag_coefficients_;   //!< delay coefficient for normal drive
  const libra::Vector<3> coasting_lag_coefficients_;  //!< delay coefficient for coasting drive(Power off)

  // Controlled Parameters
  bool drive_flag_;                    //!< Drive flag(True: Drive, False: Stop)
  double velocity_limit_rpm_;          //!< Velocity limit defined by users [RPM]
  double target_acceleration_rad_s2_;  //!< Target acceleration [rad/s2]

  // Internal variables for control delay
  std::vector<double> acceleration_delay_buffer_;  //!< Delay buffer for acceleration
  double main_routine_time_step_s_;                //!< Period of execution of main routine [sec]

  // Output at RW frame
  double angular_acceleration_rad_s2_ = 0.0;  //!< Output angular acceleration [rad/s2]
  double angular_velocity_rpm_ = 0.0;         //!< Current angular velocity [rpm]
  double angular_velocity_rad_s_ = 0.0;       //!< Current angular velocity [rad/s]

  // Output at body frame
  libra::Vector<3> output_torque_b_Nm_{0.0};      //!< Output torque in the body fixed frame [Nm]
  libra::Vector<3> angular_momentum_b_Nms_{0.0};  //!< Angular momentum of RW [Nms]

  ReactionWheelOde ode_angular_velocity_;  //!< Reaction Wheel OrdinaryDifferentialEquation
  ReactionWheelJitter rw_jitter_;          //!< RW jitter
  bool is_calculated_jitter_ = false;      //!< Flag for calculation of jitter
  bool is_logged_jitter_ = false;          //!< Flag for log output of jitter

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
