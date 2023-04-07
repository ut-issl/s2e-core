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

const size_t kMagnetometerDimension = 3;  //!< Dimension of magnetometer

/**
 * @class Magnetometer
 * @brief Class to emulate magnetometer
 */
class Magnetometer : public Component, public Sensor<kMagnetometerDimension>, public ILoggable {
 public:
  /**
   * @fn Magnetometer
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] sensor_base: Sensor base information
   * @param [in] sensor_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] geomagnetic_field: Geomagnetic environment
   */
  Magnetometer(const int prescaler, ClockGenerator* clock_generator, Sensor& sensor_base, const unsigned int sensor_id,
               const libra::Quaternion& quaternion_b2c, const GeomagneticField* geomagnetic_field);
  /**
   * @fn Magnetometer
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] sensor_base: Sensor base information
   * @param [in] sensor_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] geomagnetic_field: Geomagnetic environment
   */
  Magnetometer(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, Sensor& sensor_base, const unsigned int sensor_id,
               const libra::Quaternion& quaternion_b2c, const GeomagneticField* geomagnetic_field);
  /**
   * @fn ~Magnetometer
   * @brief Destructor
   */
  ~Magnetometer();

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
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

  /**
   * @fn GetMeasuredMagneticField_c_nT
   * @brief Return observed magnetic field on the component frame
   */
  inline const libra::Vector<kMagnetometerDimension>& GetMeasuredMagneticField_c_nT(void) const { return magnetic_field_c_nT_; }

 protected:
  libra::Vector<kMagnetometerDimension> magnetic_field_c_nT_{0.0};  //!< Observed magnetic field on the component frame [nT]
  unsigned int sensor_id_ = 0;                                      //!< Sensor ID
  libra::Quaternion quaternion_b2c_{0.0, 0.0, 0.0, 1.0};            //!< Quaternion from body frame to component frame

  const GeomagneticField* geomagnetic_field_;  //!< Geomagnetic environment
};

#endif  // S2E_COMPONENTS_REAL_AOCS_MAGNETOMETER_HPP_
