/*
 * @file reaction_wheel.hpp
 * @brief Class to emulate Reaction Wheel
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_HPP_
#define S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_HPP_

#include <library/logger/logger.hpp>
#include <library/math/vector.hpp>
#include <limits>
#include <string>
#include <vector>

#include "../../base/component.hpp"
#include "reaction_wheel_jitter.hpp"
#include "reaction_wheel_ode.hpp"

/*
 * @class ReactionWheel
 * @brief Class to emulate Reaction Wheel
 * @note For one reaction wheel
 */
class ReactionWheel : public Component, public ILoggable {
 public:
  /**
   * @fn ReactionWheel
   * @brief Constructor without power port
   * @note TODO: argument is too long
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] fast_prescaler: Frequency scale factor for fast update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: Component ID
   * @param [in] step_width_s: Step width of integration by reaction wheel ordinary differential equation [sec]
   * @param [in] jitter_update_interval_s: Update period of RW jitter [sec]
   * @param [in] rotor_inertia_kgm2: Moment of rotor_inertia_kgm2 of the RW [kgm2]
   * @param [in] max_torque_Nm: Maximum output torque [Nm]
   * @param [in] max_velocity_rpm: Maximum output angular velocity [RPM]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] position_b_m: Position of RW on the body fixed frame [m]
   * @param [in] dead_time_s: Dead time of torque output [sec]
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
  ReactionWheel(const int prescaler, const int fast_prescaler, ClockGenerator* clock_generator, const int component_id, const double step_width_s,
                const double jitter_update_interval_s, const double rotor_inertia_kgm2, const double max_torque_Nm, const double max_velocity_rpm,
                const libra::Quaternion quaternion_b2c, const libra::Vector<3> position_b_m, const double dead_time_s,
                const bool is_calc_jitter_enabled, const bool is_log_jitter_enabled,
                const std::vector<std::vector<double>> radial_force_harmonics_coefficients,
                const std::vector<std::vector<double>> radial_torque_harmonics_coefficients, const double structural_resonance_frequency_Hz,
                const double damping_factor, const double bandwidth, const bool considers_structural_resonance, const bool drive_flag = false,
                const double init_velocity_rad_s = 0.0);
  /**
   * @fn ReactionWheel
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] fast_prescaler: Frequency scale factor for fast update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] component_id: Component ID
   * @param [in] step_width_s: Step width of integration by reaction wheel ordinary differential equation [sec]
   * @param [in] jitter_update_interval_s: Update period of RW jitter [sec]
   * @param [in] rotor_inertia_kgm2: Moment of rotor_inertia_kgm2 of the RW [kgm2]
   * @param [in] max_torque_Nm: Maximum output torque [Nm]
   * @param [in] max_velocity_rpm: Maximum output angular velocity [RPM]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] position_b_m: Position of RW on the body fixed frame [m]
   * @param [in] dead_time_s: Dead time of torque output [sec]
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
  ReactionWheel(const int prescaler, const int fast_prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
                const double step_width_s, const double jitter_update_interval_s, const double rotor_inertia_kgm2, const double max_torque_Nm,
                const double max_velocity_rpm, const libra::Quaternion quaternion_b2c, const libra::Vector<3> position_b_m, const double dead_time_s,
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
  virtual std::string GetLogHeader() const override;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const override;

  // Getter
  /**
   * @fn GetOutputTorque_b_Nm
   * @brief Return output torque in the body fixed frame [Nm]
   */
  const libra::Vector<3> GetOutputTorque_b_Nm() const;
  /**
   * @fn GetJitterForce_b_N
   * @brief Return output force by jitter in the body fixed frame [N]
   */
  inline const libra::Vector<3> GetJitterForce_b_N() const { return rw_jitter_.GetJitterForce_b_N(); }
  /**
   * @fn GetDriveFlag
   * @brief Return drive flag
   */
  inline bool GetDriveFlag() const { return drive_flag_; };
  /**
   * @fn GetAngularVelocity_rad_s
   * @brief Return angular velocity of RW rotor [rad/s]
   */
  inline double GetAngularVelocity_rad_s() const { return angular_velocity_rad_s_; };
  /**
   * @fn GetAngularVelocity_rpm
   * @brief Return angular velocity of RW rotor [RPM]
   */
  inline double GetAngularVelocity_rpm() const { return angular_velocity_rpm_; };
  /**
   * @fn GetAngularMomentum_b_Nms
   * @brief Return angular momentum of RW [Nms]
   */
  inline const libra::Vector<3> GetAngularMomentum_b_Nms() const { return angular_momentum_b_Nms_; };

  // Setter
  /**
   * @fn SetTargetTorque_rw_Nm
   * @brief Set target torque on RW frame [Nm]
   */
  void SetTargetTorque_rw_Nm(const double torque_rw_Nm);
  /**
   * @fn SetTargetTorque_b_Nm
   * @brief Set target torque on body frame (opposite of RW frame) [Nm]
   */
  void SetTargetTorque_b_Nm(const double torque_b_Nm);
  /**
   * @fn SetVelocityLimit_rpm
   * @brief Set velocity limit [RPM]
   */
  void SetVelocityLimit_rpm(const double velocity_limit_rpm);
  /**
   * @fn SetDriveFlag
   * @brief Set drive flag
   */
  inline void SetDriveFlag(const bool flag) { drive_flag_ = flag; };

 protected:
  // Fixed Parameters
  const int component_id_;                  //!< Actuator ID
  const double rotor_inertia_kgm2_;         //!< Inertia of RW rotor [kgm2]
  const double max_torque_Nm_;              //!< Maximum output torque [Nm]
  const double max_velocity_rpm_;           //!< Maximum angular velocity of rotor [rpm]
  const libra::Quaternion quaternion_b2c_;  //!< Quaternion from body frame to component frame
  const libra::Vector<3> position_b_m_;     //!< Position of RW in the body fixed frame [m]
  libra::Vector<3> rotation_axis_c_;        //!< Wheel rotation axis on the component frame. Constant as (0 0 1). (Output torque is minus direction)
  libra::Vector<3> rotation_axis_b_;        //!< Wheel rotation vector in the body fixed frame.

  // Parameters for control delay
  const double step_width_s_;                      //!< step width for ReactionWheelOde [sec]
  const double dead_time_s_;                       //!< dead time [sec]
  std::vector<double> acceleration_delay_buffer_;  //!< Delay buffer for acceleration

  // Controlled Parameters
  bool drive_flag_;                    //!< Drive flag(True: Drive, False: Stop)
  double target_acceleration_rad_s2_;  //!< Target acceleration [rad/s2]

  // Output at RW frame
  double generated_angular_acceleration_rad_s2_ = 0.0;  //!< Generated acceleration [rad/s2]
  double angular_velocity_rpm_ = 0.0;                   //!< Current angular velocity [rpm]
  double angular_velocity_rad_s_ = 0.0;                 //!< Current angular velocity [rad/s]
  // Output at body frame
  libra::Vector<3> output_torque_b_Nm_{0.0};      //!< Output torque in the body fixed frame [Nm]
  libra::Vector<3> angular_momentum_b_Nms_{0.0};  //!< Angular momentum of RW [Nms]

  // ODE
  double velocity_limit_rpm_;              //!< Velocity limit defined by users [RPM]
  ReactionWheelOde ode_angular_velocity_;  //!< Reaction Wheel OrdinaryDifferentialEquation

  // RW jitter
  ReactionWheelJitter rw_jitter_;      //!< RW jitter
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

/**
 * @fn InitReactionWheel
 * @brief Initialize functions for reaction wheel without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] actuator_id: Actuator ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] compo_update_step: Component step time [sec]
 */
ReactionWheel InitReactionWheel(ClockGenerator* clock_generator, int actuator_id, std::string file_name, double compo_update_step);
/**
 * @fn InitReactionWheel
 * @brief Initialize functions for reaction wheel with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] actuator_id: Actuator ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] prop_step: Propagation step for RW dynamics [sec]
 * @param [in] compo_update_step: Component step time [sec]
 */
ReactionWheel InitReactionWheel(ClockGenerator* clock_generator, PowerPort* power_port, int actuator_id, std::string file_name,
                                double compo_update_step);

#endif  // S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_HPP_
