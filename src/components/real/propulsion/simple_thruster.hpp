/*
 * @file simple_thruster.hpp
 * @brief Component emulation of simple thruster
 */

#ifndef S2E_COMPONENTS_REAL_PROPULSION_SIMPLE_THRUSTER_HPP_
#define S2E_COMPONENTS_REAL_PROPULSION_SIMPLE_THRUSTER_HPP_

#include <dynamics/dynamics.hpp>
#include <logger/logger.hpp>
#include <math_physics/math/quaternion.hpp>
#include <math_physics/math/vector.hpp>
#include <math_physics/randomization/normal_randomization.hpp>
#include <simulation/spacecraft/structure/structure.hpp>

#include "../../base/component.hpp"

namespace s2e::components {

/*
 * @class SimpleThruster
 * @brief Component emulation of simple thruster
 */
class SimpleThruster : public Component, public logger::ILoggable {
 public:
  /**
   * @fn SimpleThruster
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: Thruster ID
   * @param [in] thruster_position_b_m: Position of thruster on the body fixed frame [m]
   * @param [in] thrust_direction_b: Direction of thrust on the body fixed frame
   * @param [in] max_magnitude_N: Maximum thrust magnitude [N]
   * @param [in] magnitude_standard_deviation_N: Standard deviation of thrust magnitude error [N]
   * @param [in] direction_standard_deviation_rad: Standard deviation of thrust direction error [rad]
   * @param [in] structure: Spacecraft structure information
   * @param [in] dynamics: Spacecraft dynamics information
   */
  SimpleThruster(const int prescaler, environment::ClockGenerator* clock_generator, const int component_id,
                 const math::Vector<3> thruster_position_b_m, const math::Vector<3> thrust_direction_b, const double max_magnitude_N,
                 const double magnitude_standard_deviation_N, const double direction_standard_deviation_rad, const spacecraft::Structure* structure,
                 const dynamics::Dynamics* dynamics);
  /**
   * @fn SimpleThruster
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] component_id: Thruster ID
   * @param [in] thruster_position_b_m: Position of thruster on the body fixed frame [m]
   * @param [in] thrust_direction_b: Direction of thrust on the body fixed frame
   * @param [in] max_magnitude_N: Maximum thrust magnitude [N]
   * @param [in] magnitude_standard_deviation_N: Standard deviation of thrust magnitude error [N]
   * @param [in] direction_standard_deviation_rad: Standard deviation of thrust direction error [rad]
   * @param [in] structure: Spacecraft structure information
   * @param [in] dynamics: Spacecraft dynamics information
   */
  SimpleThruster(const int prescaler, environment::ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
                 const math::Vector<3> thruster_position_b_m, const math::Vector<3> thrust_direction_b, const double max_magnitude_N,
                 const double magnitude_standard_deviation_N, const double direction_standard_deviation_rad, const spacecraft::Structure* structure,
                 const dynamics::Dynamics* dynamics);
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
  void MainRoutine(const int time_count) override;
  /**
   * @fn PowerOffRoutine
   * @brief Power off routine to stop force generation
   */
  void PowerOffRoutine() override;

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
   * @fn GetOutputThrust_b_N
   * @brief Return generated thrust on the body fixed frame [N]
   */
  inline const math::Vector<3> GetOutputThrust_b_N() const { return output_thrust_b_N_; };
  /**
   * @fn GetOutputTorque_b_Nm
   * @brief Return generated torque on the body fixed frame [Nm]
   */
  inline const math::Vector<3> GetOutputTorque_b_Nm() const { return output_torque_b_Nm_; };

  // Setter
  /**
   * @fn SetDuty
   * @brief Set duty
   */
  inline void SetDuty(double duty) { duty_ = duty; };

 protected:
  // parameters
  const int component_id_;                               //!< Thruster ID
  math::Vector<3> thruster_position_b_m_{0.0};           //!< Thruster position @ body frame [m]
  math::Vector<3> thrust_direction_b_{0.0};              //!< Thrust direction @ body frame
  double duty_ = 0.0;                                    //!< PWM Duty [0.0 : 1.0]
  double thrust_magnitude_max_N_ = 0.0;                  //!< Maximum thrust magnitude [N]
  double direction_noise_standard_deviation_rad_ = 0.0;  //!< Standard deviation of thrust direction error [rad]
  randomization::NormalRand magnitude_random_noise_;     //!< Normal random for thrust magnitude error
  randomization::NormalRand direction_random_noise_;     //!< Normal random for thrust direction error
  // outputs
  math::Vector<3> output_thrust_b_N_{0.0};   //!< Generated thrust on the body fixed frame [N]
  math::Vector<3> output_torque_b_Nm_{0.0};  //!< Generated torque on the body fixed frame [Nm]

  /**
   * @fn CalcThrust
   * @brief Calculate generated thrust
   */
  void CalcThrust();
  /**
   * @fn CalcTorque
   * @brief Calculate generated torque
   * @param [in] center_of_mass_b_m: Center of mass_kg position at body frame [m]
   */
  void CalcTorque(const math::Vector<3> center_of_mass_b_m);
  /**
   * @fn CalcThrustMagnitude
   * @brief Calculate thrust magnitude
   * @return Thrust magnitude
   */
  double CalcThrustMagnitude();
  /**
   * @fn CalcThrustDirection
   * @brief Calculate thrust direction
   * @return Thrust direction
   */
  math::Vector<3> CalcThrustDirection();
  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] magnitude_standard_deviation_N: Standard deviation of thrust magnitude error [N]
   * @param [in] direction_standard_deviation_rad: Standard deviation of thrust direction error [rad]
   */
  void Initialize(const double magnitude_standard_deviation_N, const double direction_standard_deviation_rad);

  const spacecraft::Structure* structure_;  //!< Spacecraft structure information
  const dynamics::Dynamics* dynamics_;      //!< Spacecraft dynamics information
};

/**
 * @fn InitSimpleThruster
 * @brief Initialize function os SimpleThruster
 * @param [in] clock_generator: Clock generator
 * @param [in] thruster_id: Thruster ID
 * @param [in] file_name: Path to initialize file
 * @param [in] structure: Spacecraft structure information
 * @param [in] dynamics: Spacecraft dynamics information
 */
SimpleThruster InitSimpleThruster(environment::ClockGenerator* clock_generator, int thruster_id, const std::string file_name,
                                  const spacecraft::Structure* structure, const dynamics::Dynamics* dynamics);
/**
 * @fn InitSimpleThruster
 * @brief Initialize function os SimpleThruster
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] thruster_id: Thruster ID
 * @param [in] file_name: Path to initialize file
 * @param [in] structure: Spacecraft structure information
 * @param [in] dynamics: Spacecraft dynamics information
 */
SimpleThruster InitSimpleThruster(environment::ClockGenerator* clock_generator, PowerPort* power_port, int thruster_id, const std::string file_name,
                                  const spacecraft::Structure* structure, const dynamics::Dynamics* dynamics);

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_REAL_PROPULSION_SIMPLE_THRUSTER_HPP_
