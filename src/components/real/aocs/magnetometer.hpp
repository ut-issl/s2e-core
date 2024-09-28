/**
 * @file magnetometer.hpp
 * @brief Class to emulate magnetometer
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_MAGNETOMETER_HPP_
#define S2E_COMPONENTS_REAL_AOCS_MAGNETOMETER_HPP_

#include <environment/local/local_environment.hpp>
#include <logger/loggable.hpp>
#include <math_physics/math/quaternion.hpp>

#include "../../base/component.hpp"
#include "../../base/sensor.hpp"

namespace s2e::components {

const size_t kMagnetometerDimension = 3;  //!< Dimension of magnetometer

/**
 * @class Magnetometer
 * @brief Class to emulate magnetometer
 */
class Magnetometer : public Component, public Sensor<kMagnetometerDimension>, public logger::ILoggable {
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
  Magnetometer(const int prescaler, environment::ClockGenerator* clock_generator, Sensor& sensor_base, const unsigned int sensor_id,
               const math::Quaternion& quaternion_b2c, const GeomagneticField* geomagnetic_field);
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
  Magnetometer(const int prescaler, environment::ClockGenerator* clock_generator, PowerPort* power_port, Sensor& sensor_base, const unsigned int sensor_id,
               const math::Quaternion& quaternion_b2c, const GeomagneticField* geomagnetic_field);
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
   * @fn GetMeasuredMagneticField_c_nT
   * @brief Return observed magnetic field on the component frame
   */
  inline const math::Vector<kMagnetometerDimension>& GetMeasuredMagneticField_c_nT(void) const { return magnetic_field_c_nT_; }

  /**
   * @fn SetConstantBiasNoise_c_nT
   * @brief Set constant bias noise at component frame [nT]
   * @param [in] constant_noise_c_nT: Constant bias noise at component frame [nT]
   */
  inline void SetConstantBiasNoise_c_nT(math::Vector<kMagnetometerDimension> constant_noise_c_nT) { bias_noise_c_ = constant_noise_c_nT; }

  /**
   * @fn AddConstantBiasNoise_c_nT
   * @brief Add constant bias noise at component frame [nT]
   * @param [in] constant_noise_c_nT: Additional constant bias noise at component frame [nT]
   */
  inline void AddConstantBiasNoise_c_nT(math::Vector<kMagnetometerDimension> constant_noise_c_nT) { bias_noise_c_ += constant_noise_c_nT; }

  /**
   * @fn GetConstantBiasNoise_c_nT
   * @brief Get constant bias noise at component frame [nT]
   */
  inline math::Vector<kMagnetometerDimension> GetConstantBiasNoise_c_nT() const { return bias_noise_c_; }

 protected:
  math::Vector<kMagnetometerDimension> magnetic_field_c_nT_{0.0};  //!< Observed magnetic field on the component frame [nT]
  unsigned int sensor_id_ = 0;                                     //!< Sensor ID
  math::Quaternion quaternion_b2c_{0.0, 0.0, 0.0, 1.0};            //!< Quaternion from body frame to component frame

  const GeomagneticField* geomagnetic_field_;  //!< Geomagnetic environment
};

/**
 * @fn InitMagnetometer
 * @brief Initialize functions for magnetometer without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] geomagnetic_field: Geomegnetic environment
 */
Magnetometer InitMagnetometer(environment::ClockGenerator* clock_generator, int sensor_id, const std::string file_name, double component_step_time_s,
                              const GeomagneticField* geomagnetic_field);
/**
 * @fn InitMagnetometer
 * @brief Initialize functions for magnetometer with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] sensor_id: Sensor ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] geomagnetic_field: Geomegnetic environment
 */
Magnetometer InitMagnetometer(environment::ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const std::string file_name,
                              double component_step_time_s, const GeomagneticField* geomagnetic_field);

} // namespace s2e::components

#endif  // S2E_COMPONENTS_REAL_AOCS_MAGNETOMETER_HPP_
