/**
 * @file gyro_sensor.hpp
 * @brief Class to emulate gyro sensor (angular velocity sensor)
 */

#ifndef S2E_COMPONENTS_AOCS_GYRO_SENSOR_HPP_
#define S2E_COMPONENTS_AOCS_GYRO_SENSOR_HPP_

#include <dynamics/dynamics.hpp>
#include <interface/log_output/loggable.hpp>
#include <library/math/quaternion.hpp>

#include "../base/component.hpp"
#include "../base/sensor_base.hpp"

const size_t kGyroDim = 3;  //!< Dimension of gyro sensor

/**
 * @class Gyro
 * @brief Class to emulate gyro sensor
 */
class Gyro : public ComponentBase, public SensorBase<kGyroDim>, public ILoggable {
 public:
  /**
   * @fn Gyro
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] sensor_base: Sensor base information
   * @param [in] sensor_id: Sensor ID
   * @param [in] q_b2c: Quaternion from body frame to component frame
   * @param [in] dynamics: Dynamics information
   */
  Gyro(const int prescaler, ClockGenerator* clock_gen, SensorBase& sensor_base, const int sensor_id, const libra::Quaternion& q_b2c,
       const Dynamics* dynamics);
  /**
   * @fn Gyro
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] power_port: Power port
   * @param [in] sensor_base: Sensor base information
   * @param [in] sensor_id: Sensor ID
   * @param [in] q_b2c: Quaternion from body frame to component frame
   * @param [in] dynamics: Dynamics information
   */
  Gyro(const int prescaler, ClockGenerator* clock_gen, PowerPort* power_port, SensorBase& sensor_base, const int sensor_id,
       const libra::Quaternion& q_b2c, const Dynamics* dynamics);
  /**
   * @fn ~Gyro
   * @brief Destructor
   */
  ~Gyro();

  // Override functions for ComponentBase
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(int count) override;

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

  /**
   * @fn GetOmegaC
   * @brief Return observed angular velocity of the component frame with respect to the inertial frame
   */
  inline const libra::Vector<kGyroDim>& GetOmegaC(void) const { return omega_c_; }

 protected:
  libra::Vector<kGyroDim> omega_c_{0.0};         //!< Observed angular velocity of the component frame with respect to the inertial frame [rad/s]
  int sensor_id_ = 0;                            //!< Sensor ID
  libra::Quaternion q_b2c_{0.0, 0.0, 0.0, 1.0};  //!< Quaternion from body frame to component frame

  const Dynamics* dynamics_;  //!< Dynamics information
};

#endif  // S2E_COMPONENTS_AOCS_GYRO_SENSOR_HPP_
