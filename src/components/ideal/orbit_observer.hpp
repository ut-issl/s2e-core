/*
 * @file orbit_observer.hpp
 * @brief Ideal component which can observe orbit
 */

#ifndef S2E_COMPONENTS_IDEAL_ORBIT_OBSERVER_HPP_
#define S2E_COMPONENTS_IDEAL_ORBIT_OBSERVER_HPP_

#include <dynamics/orbit/orbit.hpp>
#include <logger/loggable.hpp>
#include <math_physics/math/vector.hpp>
#include <math_physics/randomization/normal_randomization.hpp>

#include "../base/component.hpp"

namespace s2e::components {

/**
 * @enum NoiseFrame
 * @brief Noise definition frame
 */
enum class NoiseFrame {
  kInertial,  //!< Inertial frame
  kRtn,       //!< RTN frame
};

/*
 * @class OrbitObserver
 * @brief Ideal component which can observe orbit
 */
class OrbitObserver : public Component, public logger::ILoggable {
 public:
  /**
   * @fn OrbitObserver
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] noise_frame: Error frame definition
   * @param [in] error_standard_deviation: Position and Velocity standard deviation noise [m, m/s]
   * @param [in] orbit: Orbit information
   */
  OrbitObserver(const int prescaler, environment::ClockGenerator* clock_generator, const NoiseFrame noise_frame,
                const math::Vector<6> error_standard_deviation, const dynamics::orbit::Orbit& orbit);

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
   * @fn GetPosition_i_m
   * @brief Return observed position
   */
  inline const math::Vector<3> GetPosition_i_m() const { return observed_position_i_m_; };

  /**
   * @fn GetVelocity_i_m_s
   * @brief Return observed velocity
   */
  inline const math::Vector<3> GetVelocity_i_m_s() const { return observed_velocity_i_m_s_; };

 protected:
  math::Vector<3> observed_position_i_m_{0.0};    //!< Observed position @ inertial frame [m]
  math::Vector<3> observed_velocity_i_m_s_{0.0};  //!< Observed velocity @ inertial frame [m/s]

  NoiseFrame noise_frame_;                            //!< Noise definition frame
  randomization::NormalRand normal_random_noise_[6];  //!< Position and Velocity noise [m, m/s]

  // Observed variables
  const dynamics::orbit::Orbit& orbit_;  //!< Orbit information
};

/**
 * @fn SetNoiseFrame
 * @brief Set NoiseFrame by string
 * @param [in] noise_frame: Noise frame name
 * @return noise frame
 */
NoiseFrame SetNoiseFrame(const std::string noise_frame);

/**
 * @fn InitStarSensor
 * @brief Initialize functions for StarSensor without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] file_name: Path to the initialize file
 * @param [in] orbit: Orbit information
 */
OrbitObserver InitializeOrbitObserver(environment::ClockGenerator* clock_generator, const std::string file_name, const dynamics::orbit::Orbit& orbit);

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_IDEAL_ORBIT_OBSERVER_HPP_
