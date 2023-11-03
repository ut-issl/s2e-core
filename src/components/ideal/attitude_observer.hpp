/*
 * @file attitude_observer.hpp
 * @brief Ideal component which can observe attitude
 */

#ifndef S2E_COMPONENTS_IDEAL_ATTITUDE_OBSERVER_HPP_
#define S2E_COMPONENTS_IDEAL_ATTITUDE_OBSERVER_HPP_

#include <dynamics/attitude/attitude.hpp>
#include <library/logger/loggable.hpp>
#include <library/math/quaternion.hpp>
#include <library/randomization/normal_randomization.hpp>

#include "../base/component.hpp"

/*
 * @class AttitudeObserver
 * @brief Ideal component which can observe attitude
 */
class AttitudeObserver : public Component, public ILoggable {
 public:
  /**
   * @fn AttitudeObserver
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] attitude: Attitude information
   */
  AttitudeObserver(const int prescaler, ClockGenerator* clock_generator, const double standard_deviation_rad, const Attitude& attitude);

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

  /**
   * @fn GetQuaternion_i2c
   * @brief Return observed quaternion from the inertial frame to the component frame
   */
  inline const libra::Quaternion GetQuaternion_i2b() const { return observed_quaternion_i2b_; };

 protected:
  libra::Quaternion observed_quaternion_i2b_ = {0.0, 0.0, 0.0, 1.0};  //!< Observed quaternion

  libra::NormalRand angle_noise_;      //!< Normal random for magnitude noise
  libra::NormalRand direction_noise_;  //!< Normal random for direction noise

  const Attitude& attitude_;  //!< Attitude information
};

/**
 * @fn InitializeAttitudeObserver
 * @brief Initialize functions for AttitudeObserver
 * @param [in] clock_generator: Clock generator
 * @param [in] file_name: Path to the initialize file
 * @param [in] attitude: Attitude information
 */
AttitudeObserver InitializeAttitudeObserver(ClockGenerator* clock_generator, const std::string file_name, const Attitude& attitude);

#endif  // S2E_COMPONENTS_IDEAL_ATTITUDE_OBSERVER_HPP_
