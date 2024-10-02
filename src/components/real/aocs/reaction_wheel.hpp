/*
 * @file reaction_wheel.hpp
 * @brief Class to emulate Reaction Wheel
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_HPP_
#define S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_HPP_

#include <limits>
#include <logger/logger.hpp>
#include <math_physics/control_utilities/first_order_lag.hpp>
#include <math_physics/math/vector.hpp>
#include <string>
#include <vector>

#include "../../base/component.hpp"
#include "reaction_wheel_jitter.hpp"
#include "reaction_wheel_ode.hpp"

namespace s2e::components {

/*
 * @class ReactionWheel
 * @brief Class to emulate Reaction Wheel
 * @note For one reaction wheel
 */
class ReactionWheel : public Component, public logger::ILoggable {
 public:
  /**
   * @fn ReactionWheel
   * @brief Constructor without power port
   * @note TODO: argument is too long
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: Component ID
   * @param [in] step_width_s: Step width of integration by reaction wheel ordinary differential equation [sec]
   * @param [in] rotor_inertia_kgm2: Moment of rotor_inertia_kgm2 of the RW [kgm2]
   * @param [in] max_torque_Nm: Maximum output torque [Nm]
   * @param [in] max_velocity_rpm: Maximum output angular velocity [RPM]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] position_b_m: Position of RW on the body fixed frame [m]
   * @param [in] dead_time_s: Dead time of torque output [sec]
   * @param [in] time_constant_s: First order lag time constant [sec]
   * @param [in] friction_coefficients: Friction coefficients
   * @param [in] stop_limit_angular_velocity_rad_s: Angular velocity stop limit by friction [rad/s]
   * @param [in] is_calc_jitter_enabled: Enable flag to calculate RW jitter
   * @param [in] is_log_jitter_enabled: Enable flag to log output RW jitter
   * @param [in] fast_prescaler: Frequency scale factor for fast update
   * @param [in] rw_jitter: RW jitter
   * @param [in] drive_flag: RW drive flag
   * @param [in] init_velocity_rad_s: Initial value of angular velocity of RW
   */
  ReactionWheel(const int prescaler, environment::ClockGenerator* clock_generator, const int component_id, const double step_width_s,
                const double rotor_inertia_kgm2, const double max_torque_Nm, const double max_velocity_rpm, const math::Quaternion quaternion_b2c,
                const math::Vector<3> position_b_m, const double dead_time_s, const double time_constant_s,
                const std::vector<double> friction_coefficients, const double stop_limit_angular_velocity_rad_s, const bool is_calc_jitter_enabled,
                const bool is_log_jitter_enabled, const int fast_prescaler, ReactionWheelJitter& rw_jitter, const bool drive_flag = false,
                const double init_velocity_rad_s = 0.0);
  /**
   * @fn ReactionWheel
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] component_id: Component ID
   * @param [in] step_width_s: Step width of integration by reaction wheel ordinary differential equation [sec]
   * @param [in] rotor_inertia_kgm2: Moment of rotor_inertia_kgm2 of the RW [kgm2]
   * @param [in] max_torque_Nm: Maximum output torque [Nm]
   * @param [in] max_velocity_rpm: Maximum output angular velocity [RPM]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] position_b_m: Position of RW on the body fixed frame [m]
   * @param [in] dead_time_s: Dead time of torque output [sec]
   * @param [in] time_constant_s: First order lag time constant [sec]
   * @param [in] friction_coefficients: Friction coefficients
   * @param [in] stop_limit_angular_velocity_rad_s: Angular velocity stop limit by friction [rad/s]
   * @param [in] is_calc_jitter_enabled: Enable flag to calculate RW jitter
   * @param [in] is_log_jitter_enabled: Enable flag to log output RW jitter
   * @param [in] fast_prescaler: Frequency scale factor for fast update
   * @param [in] rw_jitter: RW jitter
   * @param [in] drive_flag: RW drive flag
   * @param [in] init_velocity_rad_s: Initial value of angular velocity of RW [rad/s]
   */
  ReactionWheel(const int prescaler, environment::ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
                const double step_width_s, const double rotor_inertia_kgm2, const double max_torque_Nm, const double max_velocity_rpm,
                const math::Quaternion quaternion_b2c, const math::Vector<3> position_b_m, const double dead_time_s, const double time_constant_s,
                const std::vector<double> friction_coefficients, const double stop_limit_angular_velocity_rad_s, const bool is_calc_jitter_enabled,
                const bool is_log_jitter_enabled, const int fast_prescaler, ReactionWheelJitter& rw_jitter, const bool drive_flag = false,
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

  // Override logger::ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of logger::ILoggable
   */
  virtual std::string GetLogHeader() const override;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of logger::ILoggable
   */
  virtual std::string GetLogValue() const override;

  // Getter
  /**
   * @fn GetOutputTorque_b_Nm
   * @brief Return output torque in the body fixed frame [Nm]
   */
  const math::Vector<3> GetOutputTorque_b_Nm() const;
  /**
   * @fn GetJitterForce_b_N
   * @brief Return output force by jitter in the body fixed frame [N]
   */
  inline const math::Vector<3> GetJitterForce_b_N() const;
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
  inline const math::Vector<3> GetAngularMomentum_b_Nms() const { return angular_momentum_b_Nms_; };

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
  const int component_id_;                 //!< Actuator ID
  const double rotor_inertia_kgm2_;        //!< Inertia of RW rotor [kgm2]
  const double max_torque_Nm_;             //!< Maximum output torque [Nm]
  const double max_velocity_rpm_;          //!< Maximum angular velocity of rotor [rpm]
  const math::Quaternion quaternion_b2c_;  //!< Quaternion from body frame to component frame
  const math::Vector<3> position_b_m_;     //!< Position of RW in the body fixed frame [m]
  math::Vector<3> rotation_axis_c_;        //!< Wheel rotation axis on the component frame. Constant as (0 0 1). (Output torque is minus direction)
  math::Vector<3> rotation_axis_b_;        //!< Wheel rotation vector in the body fixed frame.

  // Parameters for control delay
  const double step_width_s_;                                     //!< step width for ReactionWheelOde [sec]
  const double dead_time_s_;                                      //!< dead time [sec]
  std::vector<double> acceleration_delay_buffer_;                 //!< Delay buffer for acceleration
  control_utilities::FirstOrderLag delayed_acceleration_rad_s2_;  //!< Delayed acceleration [rad/s2]

  // Coasting friction
  // f_rad_s2 = v_rad_s * coefficients(0) + (v_rad_s)^2 * coefficients(1) + ...
  std::vector<double> coasting_friction_coefficients_;  //!< Friction coefficients for coasting
  double stop_limit_angular_velocity_rad_s_ = 0.1;      //!< Angular velocity stop limit by friction [rad/s]

  // Controlled Parameters
  bool drive_flag_;                    //!< Drive flag (True: Drive, False: Stop)
  double target_acceleration_rad_s2_;  //!< Target acceleration [rad/s2]

  // Output at RW frame
  double generated_angular_acceleration_rad_s2_ = 0.0;  //!< Generated acceleration [rad/s2]
  double angular_velocity_rpm_ = 0.0;                   //!< Current angular velocity [rpm]
  double angular_velocity_rad_s_ = 0.0;                 //!< Current angular velocity [rad/s]
  // Output at body frame
  math::Vector<3> output_torque_b_Nm_{0.0};      //!< Output torque in the body fixed frame [Nm]
  math::Vector<3> angular_momentum_b_Nms_{0.0};  //!< Angular momentum of RW [Nms]

  // ODE
  double velocity_limit_rpm_;              //!< Velocity limit defined by users [RPM]
  ReactionWheelOde ode_angular_velocity_;  //!< Reaction Wheel OrdinaryDifferentialEquation

  // RW jitter
  ReactionWheelJitter& rw_jitter_;     //!< RW jitter
  bool is_calculated_jitter_ = false;  //!< Flag for calculation of jitter
  bool is_logged_jitter_ = false;      //!< Flag for log output of jitter

  // Local functions
  /**
   * @fn CalcTorque
   * @brief Calculation of generated torque
   */
  math::Vector<3> CalcTorque();
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
 * @param [in] compo_update_step_s: Component step time [sec]
 */
ReactionWheel InitReactionWheel(environment::ClockGenerator* clock_generator, int actuator_id, std::string file_name, double compo_update_step_s);
/**
 * @fn InitReactionWheel
 * @brief Initialize functions for reaction wheel with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] actuator_id: Actuator ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] prop_step: Propagation step for RW dynamics [sec]
 * @param [in] compo_update_step_s: Component step time [sec]
 */
ReactionWheel InitReactionWheel(environment::ClockGenerator* clock_generator, PowerPort* power_port, int actuator_id, std::string file_name,
                                double compo_update_step_s);

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_HPP_
