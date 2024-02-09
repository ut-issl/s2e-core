/**
 * @file gyro_sensor.hpp
 * @brief Class to emulate gyro sensor (angular velocity sensor)
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_GYRO_SENSOR_HPP_
#define S2E_COMPONENTS_REAL_AOCS_GYRO_SENSOR_HPP_

#include <dynamics/dynamics.hpp>
#include <logger/loggable.hpp>
#include <library/math/quaternion.hpp>

#include "../../base/component.hpp"
#include "../../base/sensor.hpp"

const size_t kGyroDimension = 3;  //!< Dimension of gyro sensor

/**
 * @class GyroSensor
 * @brief Class to emulate gyro sensor
 */
class GyroSensor : public Component, public Sensor<kGyroDimension>, public ILoggable {
 public:
  /**
   * @fn GyroSensor
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] sensor_base: Sensor base information
   * @param [in] sensor_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] dynamics: Dynamics information
   */
  GyroSensor(const int prescaler, ClockGenerator* clock_generator, Sensor& sensor_base, const unsigned int sensor_id,
             const libra::Quaternion& quaternion_b2c, const Dynamics* dynamics);
  /**
   * @fn GyroSensor
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] sensor_base: Sensor base information
   * @param [in] sensor_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] dynamics: Dynamics information
   */
  GyroSensor(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, Sensor& sensor_base, const unsigned int sensor_id,
             const libra::Quaternion& quaternion_b2c, const Dynamics* dynamics);
  /**
   * @fn ~GyroSensor
   * @brief Destructor
   */
  ~GyroSensor();

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
   * @fn GetMeasuredAngularVelocity_c_rad_s
   * @brief Return observed angular velocity of the component frame with respect to the inertial frame
   */
  inline const libra::Vector<kGyroDimension>& GetMeasuredAngularVelocity_c_rad_s(void) const { return angular_velocity_c_rad_s_; }

 protected:
  libra::Vector<kGyroDimension> angular_velocity_c_rad_s_{
      0.0};                     //!< Observed angular velocity of the component frame with respect to the inertial frame [rad/s]
  unsigned int sensor_id_ = 0;  //!< Sensor ID
  libra::Quaternion quaternion_b2c_{0.0, 0.0, 0.0, 1.0};  //!< Quaternion from body frame to component frame

  const Dynamics* dynamics_;  //!< Dynamics information
};

/**
 * @fn InitGyroSensor
 * @brief Initialize functions for gyro sensor without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] file_name: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 */
GyroSensor InitGyroSensor(ClockGenerator* clock_generator, int sensor_id, const std::string file_name, double component_step_time_s,
                          const Dynamics* dynamics);
/**
 * @fn InitGyroSensor
 * @brief Initialize functions for gyro sensor with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] sensor_id: Sensor ID
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] file_name: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 */
GyroSensor InitGyroSensor(ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const std::string file_name,
                          double component_step_time_s, const Dynamics* dynamics);

#endif  // S2E_COMPONENTS_REAL_AOCS_GYRO_SENSOR_HPP_
