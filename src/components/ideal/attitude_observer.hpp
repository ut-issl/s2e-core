/*
 * @file attitude_observer.hpp
 * @brief Ideal component which can observe attitude
 */

#ifndef S2E_COMPONENTS_IDEAL_ATTITUDE_OBSERVER_HPP_
#define S2E_COMPONENTS_IDEAL_ATTITUDE_OBSERVER_HPP_

#include <dynamics/attitude/attitude.hpp>
#include <logger/loggable.hpp>
#include <math_physics/math/quaternion.hpp>
#include <math_physics/randomization/normal_randomization.hpp>

#include "../base/component.hpp"

namespace s2e::components {

/*
 * @class AttitudeObserver
 * @brief Ideal component which can observe attitude
 */
class AttitudeObserver : public Component, public logger::ILoggable {
 public:
  /**
   * @fn AttitudeObserver
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] attitude: dynamics::attitude::Attitude information
   */
  AttitudeObserver(const int prescaler, environment::ClockGenerator* clock_generator, const double standard_deviation_rad, const dynamics::attitude::Attitude& attitude);

  /**
   * @fn ~AttitudeObserver
   * @brief Destructor
   */
  ~AttitudeObserver() {}

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count) override;

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

  /**
   * @fn GetQuaternion_i2c
   * @brief Return observed quaternion from the inertial frame to the body-fixed frame
   */
  inline const math::Quaternion GetQuaternion_i2b() const { return observed_quaternion_i2b_; };

 protected:
  math::Quaternion observed_quaternion_i2b_ = {0.0, 0.0, 0.0, 1.0};  //!< Observed quaternion

  s2e::randomization::NormalRand angle_noise_;      //!< Normal random for magnitude noise
  s2e::randomization::NormalRand direction_noise_;  //!< Normal random for direction noise

  const dynamics::attitude::Attitude& attitude_;  //!< dynamics::attitude::Attitude information
};

/**
 * @fn InitializeAttitudeObserver
 * @brief Initialize functions for AttitudeObserver
 * @param [in] clock_generator: Clock generator
 * @param [in] file_name: Path to the initialize file
 * @param [in] attitude: dynamics::attitude::Attitude information
 */
AttitudeObserver InitializeAttitudeObserver(environment::ClockGenerator* clock_generator, const std::string file_name, const dynamics::attitude::Attitude& attitude);

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_IDEAL_ATTITUDE_OBSERVER_HPP_
