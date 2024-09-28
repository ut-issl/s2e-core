/*
 * @file angular_velocity_observer.hpp
 * @brief Ideal component which can observe angular velocity
 */

#ifndef S2E_COMPONENTS_IDEAL_ANGULAR_VELOCITY_OBSERVER_HPP_
#define S2E_COMPONENTS_IDEAL_ANGULAR_VELOCITY_OBSERVER_HPP_

#include <dynamics/dynamics.hpp>
#include <logger/loggable.hpp>

#include "../base/component.hpp"
#include "../base/sensor.hpp"

namespace s2e::components {

/*
 * @class AngularVelocityObserver
 * @brief Ideal component which can observe angular velocity
 */
class AngularVelocityObserver : public Component, public Sensor<3>, public logger::ILoggable {
 public:
  /**
   * @fn AngularVelocityObserver
   * @brief Constructor
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] sensor_base: Sensor base information
   * @param [in] dynamics: Dynamics information
   */
  AngularVelocityObserver(const int prescaler, environment::ClockGenerator* clock_generator, Sensor& sensor_base, const dynamics::attitude::Attitude& attitude);
  /**
   * @fn ~AngularVelocityObserver
   * @brief Destructor
   */
  ~AngularVelocityObserver() {}

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
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

  // Getter
  /**
   * @fn GetAngularVelocity_b_rad_s
   * @brief Return observed angular velocity
   */
  inline math::Vector<3> GetAngularVelocity_b_rad_s() const { return angular_velocity_b_rad_s_; }

 protected:
  math::Vector<3> angular_velocity_b_rad_s_{0.0};  //!< Observed angular velocity [rad/s]
  const dynamics::attitude::Attitude& attitude_;                       //!< Dynamics information
};

/**
 * @fn InitializeAngularVelocityObserver
 * @brief Initialize function for AngularVelocityObserver
 * @param [in] clock_generator: Clock generator
 * @param [in] file_name: Path to the initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] dynamics: Dynamics information
 */
AngularVelocityObserver InitializeAngularVelocityObserver(environment::ClockGenerator* clock_generator, const std::string file_name, double component_step_time_s,
                                                          const dynamics::attitude::Attitude& attitude);

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_IDEAL_ANGULAR_VELOCITY_OBSERVER_HPP_
