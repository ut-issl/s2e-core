/*
 * @file orbit_observer.hpp
 * @brief Ideal component which can observe orbit
 */

#ifndef S2E_COMPONENTS_IDEAL_ORBIT_OBSERVER_HPP_
#define S2E_COMPONENTS_IDEAL_ORBIT_OBSERVER_HPP_

#include <dynamics/orbit/orbit.hpp>
#include <library/logger/loggable.hpp>
#include <library/math/vector.hpp>
#include <library/randomization/normal_randomization.hpp>

#include "../base/component.hpp"

/*
 * @class OrbitObserver
 * @brief Ideal component which can observe orbit
 */
class OrbitObserver : public Component, public ILoggable {
 public:
  /**
   * @fn OrbitObserver
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] orbit: Orbit information
   */
  OrbitObserver(const int prescaler, ClockGenerator* clock_generator, const Orbit& orbit);

  /**
   * @fn ~AttitudeObserver
   * @brief Destructor
   */
  ~OrbitObserver() {}

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
   * @fn GetPosition_i_m
   * @brief Return observed quaternion from the inertial frame to the component frame
   */
  inline const libra::Vector<3> GetPosition() const { return observed_position_i_m_; };

 protected:
  libra::Vector<3> observed_position_i_m_{0.0};  //!< Observed position @ inertial frame [m]

  // Observed variables
  const Orbit& orbit_;  //!< Orbit information
};

/**
 * @fn InitStarSensor
 * @brief Initialize functions for StarSensor without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] file_name: Path to the initialize file
 * @param [in] orbit: Orbit information
 */
OrbitObserver InitializeOrbitObserver(ClockGenerator* clock_generator, const std::string file_name, const Orbit& orbit);

#endif  // S2E_COMPONENTS_IDEAL_ORBIT_OBSERVER_HPP_
