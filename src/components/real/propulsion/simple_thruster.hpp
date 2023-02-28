/*
 * @file simple_thruster.hpp
 * @brief Component emulation of simple thruster
 */

#ifndef S2E_COMPONENTS_REAL_PROPULSION_SIMPLE_THRUSTER_HPP_
#define S2E_COMPONENTS_REAL_PROPULSION_SIMPLE_THRUSTER_HPP_

#include <dynamics/dynamics.hpp>
#include <library/logger/logger.hpp>
#include <library/math/quaternion.hpp>
#include <library/math/vector.hpp>
#include <library/randomization/normal_randomization.hpp>
#include <simulation/spacecraft/structure/structure.hpp>

#include "../../base/component.hpp"

/*
 * @class SimpleThruster
 * @brief Component emulation of simple thruster
 */
class SimpleThruster : public Component, public ILoggable {
 public:
  /**
   * @fn SimpleThruster
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: Thruster ID
   * @param [in] thruster_pos_b: Position of thruster on the body fixed frame [m]
   * @param [in] thrust_dir_b: Direction of thrust on the body fixed frame
   * @param [in] max_mag: Maximum thrust magnitude [N]
   * @param [in] mag_err: Standard deviation of thrust magnitude error [N]
   * @param [in] dir_err: Standard deviation of thrust direction error [rad]
   * @param [in] structure: Spacecraft structure information
   * @param [in] dynamics: Spacecraft dynamics information
   */
  SimpleThruster(const int prescaler, ClockGenerator* clock_generator, const int component_id, const Vector<3> thruster_pos_b,
                 const Vector<3> thrust_dir_b, const double max_mag, const double mag_err, const double dir_err, const Structure* structure,
                 const Dynamics* dynamics);
  /**
   * @fn SimpleThruster
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] component_id: Thruster ID
   * @param [in] thruster_pos_b: Position of thruster on the body fixed frame [m]
   * @param [in] thrust_dir_b: Direction of thrust on the body fixed frame
   * @param [in] max_mag: Maximum thrust magnitude [N]
   * @param [in] mag_err: Standard deviation of thrust magnitude error [N]
   * @param [in] dir_err: Standard deviation of thrust direction error [rad]
   * @param [in] structure: Spacecraft structure information
   * @param [in] dynamics: Spacecraft dynamics information
   */
  SimpleThruster(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id, const Vector<3> thruster_pos_b,
                 const Vector<3> thrust_dir_b, const double max_mag, const double mag_err, const double dir_err, const Structure* structure,
                 const Dynamics* dynamics);
  /**
   * @fn ~SimpleThruster
   * @brief Destructor
   */
  ~SimpleThruster();

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
   */
  void MainRoutine(int count);
  /**
   * @fn PowerOffRoutine
   * @brief Power off routine to stop force generation
   */
  void PowerOffRoutine() override;

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
   * @fn GetThrustB
   * @brief Return generated thrust on the body fixed frame [N]
   */
  inline const Vector<3> GetThrustB() const { return output_thrust_b_N_; };
  /**
   * @fn GetTorqueB
   * @brief Return generated torque on the body fixed frame [Nm]
   */
  inline const Vector<3> GetTorqueB() const { return output_torque_b_Nm_; };

  // Setter
  /**
   * @fn SetDuty
   * @brief Set duty
   */
  inline void SetDuty(double duty) { duty_ = duty; };

 protected:
  // parameters
  const int component_id_;                               //!< Thruster ID
  Vector<3> thruster_position_b_m_{0.0};                 //!< Thruster position @ body frame [m]
  Vector<3> thrust_direction_b_{0.0};                    //!< Thrust direction @ body frame
  double duty_ = 0.0;                                    //!< PWM Duty [0.0 : 1.0]
  double thrust_magnitude_max_N_ = 0.0;                  //!< Maximum thrust magnitude [N]
  double direction_noise_standard_deviation_rad_ = 0.0;  //!< Standard deviation of thrust direction error [rad]
  libra::NormalRand magnitude_random_noise_;             //!< Normal random for thrust magnitude error
  libra::NormalRand direction_random_noise_;             //!< Normal random for thrust direction error
  // outputs
  Vector<3> output_thrust_b_N_{0.0};   //!< Generated thrust on the body fixed frame [N]
  Vector<3> output_torque_b_Nm_{0.0};  //!< Generated torque on the body fixed frame [Nm]

  /**
   * @fn CalcThrust
   * @brief Calculate generated thrust
   */
  void CalcThrust();
  /**
   * @fn CalcTorque
   * @brief Calculate generated torque
   */
  void CalcTorque(Vector<3> center);
  /**
   * @fn CalcThrustMagnitude
   * @brief Calculate thrust magnitude
   * @return Thrust magnitude
   */
  double CalcThrustMagnitude();
  /**
   * @fn CalcThrustDir
   * @brief Calculate thrust direction
   * @return Thrust direction
   */
  Vector<3> CalcThrustDir();
  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] mag_err: Standard deviation of thrust magnitude error [N]
   * @param [in] dir_err: Standard deviation of thrust direction error [rad]
   */
  void Initialize(const double mag_err, const double dir_err);

  const Structure* structure_;  //!< Spacecraft structure information
  const Dynamics* dynamics_;    //!< Spacecraft dynamics information
};

#endif  // S2E_COMPONENTS_REAL_PROPULSION_SIMPLE_THRUSTER_HPP_
