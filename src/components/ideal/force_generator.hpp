/*
 * @file force_generator.hpp
 * @brief Ideal component which can generate force for control algorithm test
 */

#ifndef S2E_COMPONENTS_IDEAL_FORCE_GENERATOR_HPP_
#define S2E_COMPONENTS_IDEAL_FORCE_GENERATOR_HPP_

#include <components/base/component.hpp>
#include <dynamics/dynamics.hpp>
#include <library/logger/logger.hpp>
#include <library/math/vector.hpp>
#include <library/randomization/normal_randomization.hpp>

/*
 * @class ForceGenerator
 * @brief Ideal component which can generate for control algorithm test
 */
class ForceGenerator : public Component, public ILoggable {
 public:
  /**
   * @fn ForceGenerator
   * @brief Constructor
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] magnitude_error_standard_deviation_N: Standard deviation of magnitude error [N]
   * @param [in] direction_error_standard_deviation_rad: Standard deviation of direction error [rad]
   * @param [in] dynamics: Dynamics information
   */
  ForceGenerator(const int prescaler, ClockGenerator* clock_generator, const double magnitude_error_standard_deviation_N,
                 const double direction_error_standard_deviation_rad, const Dynamics* dynamics);
  /**
   * @fn ~ForceGenerator
   * @brief Destructor
   */
  ~ForceGenerator();

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
   */
  void MainRoutine(const int time_count);
  /**
   * @fn PowerOffRoutine
   * @brief Power off routine to stop force generation
   */
  void PowerOffRoutine();

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
   * @fn GetGeneratedForce_b_N
   * @brief Return generated force in the body fixed frame [N]
   */
  inline const Vector<3> GetGeneratedForce_b_N() const { return generated_force_b_N_; };
  /**
   * @fn GetGeneratedForce_i_N
   * @brief Return generated force in the inertial frame [N]
   */
  inline const Vector<3> GetGeneratedForce_i_N() const { return generated_force_i_N_; };
  /**
   * @fn GetGeneratedForce_rtn_N
   * @brief Return generated force in the RTN frame [N]
   */
  inline const Vector<3> GetGeneratedForce_rtn_N() const { return generated_force_rtn_N_; };

  // Setter
  /**
   * @fn SetForce_b_N
   * @brief Set ordered force in the body fixed frame [N]
   */
  inline void SetForce_b_N(const libra::Vector<3> force_b_N) { ordered_force_b_N_ = force_b_N; };
  /**
   * @fn SetForce_i_N
   * @brief Set ordered force in the inertial frame [N]
   */
  void SetForce_i_N(const libra::Vector<3> force_i_N);
  /**
   * @fn SetForce_rtn_N
   * @brief Set ordered force in the RTN frame [N]
   */
  void SetForce_rtn_N(const libra::Vector<3> force_rtn_N);

 protected:
  libra::Vector<3> ordered_force_b_N_{0.0};      //!< Ordered force in the body fixed frame [N]
  libra::Vector<3> generated_force_b_N_{0.0};    //!< Generated force in the body fixed frame [N]
  libra::Vector<3> generated_force_i_N_{0.0};    //!< Generated force in the inertial frame [N]
  libra::Vector<3> generated_force_rtn_N_{0.0};  //!< Generated force in the RTN frame [N]

  // Noise
  libra::NormalRand magnitude_noise_;              //!< Normal random for magnitude noise
  libra::NormalRand direction_noise_;              //!< Normal random for direction noise
  double direction_error_standard_deviation_rad_;  //!< Standard deviation of direction error [rad]

  /**
   * @fn GenerateDirectionNoiseQuaternion
   * @brief Generate direction noise quaternion
   * @param [in] true_direction: True direction
   * @param [in] error_standard_deviation_rad: Standard deviation of direction error [rad]
   */
  libra::Quaternion GenerateDirectionNoiseQuaternion(libra::Vector<3> true_direction, const double error_standard_deviation_rad);

  const Dynamics* dynamics_;  //!< Spacecraft dynamics information
};

/**
 * @fn InitializeForceGenerator
 * @brief Initialize function for ForceGenerator
 * @param [in] clock_generator: Clock generator
 * @param [in] file_name: Path to initialize file
 * @param [in] dynamics: Dynamics information
 */
ForceGenerator InitializeForceGenerator(ClockGenerator* clock_generator, const std::string file_name, const Dynamics* dynamics);

#endif  // S2E_COMPONENTS_IDEAL_FORCE_GENERATOR_HPP_
