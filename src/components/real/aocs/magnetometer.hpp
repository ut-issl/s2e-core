/**
 * @file magnetometer.hpp
 * @brief Class to emulate magnetometer
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_MAGNETOMETER_HPP_
#define S2E_COMPONENTS_REAL_AOCS_MAGNETOMETER_HPP_

#include <environment/local/local_environment.hpp>
#include <library/logger/loggable.hpp>
#include <library/math/quaternion.hpp>

#include "../../base/component.hpp"
#include "../../base/sensor.hpp"

const size_t kMagDim = 3;  //!< Dimension of magnetometer

/**
 * @class MagSensor
 * @brief Class to emulate magnetometer
 */
class MagSensor : public Component, public Sensor<kMagDim>, public ILoggable {
 public:
  /**
   * @fn MagSensor
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] sensor_base: Sensor base information
   * @param [in] sensor_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] magnet: Geomagnetic environment
   */
  MagSensor(const int prescaler, ClockGenerator* clock_generator, Sensor& sensor_base, const int sensor_id, const libra::Quaternion& quaternion_b2c,
            const GeomagneticField* magnet);
  /**
   * @fn MagSensor
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] sensor_base: Sensor base information
   * @param [in] sensor_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] magnet: Geomagnetic environment
   */
  MagSensor(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, Sensor& sensor_base, const int sensor_id,
            const libra::Quaternion& quaternion_b2c, const GeomagneticField* magnet);
  /**
   * @fn ~MagSensor
   * @brief Destructor
   */
  ~MagSensor();

  // Override functions for Component
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
   * @fn GetMagC
   * @brief Return observed magnetic field on the component frame
   */
  inline const libra::Vector<kMagDim>& GetMagC(void) const { return mag_c_; }

 protected:
  libra::Vector<kMagDim> mag_c_{0.0};            // observed magnetic field on the component frame [nT]
  int sensor_id_ = 0;                            //!< Sensor ID
  libra::Quaternion q_b2c_{0.0, 0.0, 0.0, 1.0};  //!< Quaternion from body frame to component frame

  const GeomagneticField* magnet_;  //!< Geomagnetic environment
};

#endif  // S2E_COMPONENTS_REAL_AOCS_MAGNETOMETER_HPP_
