/**
 * @file MagSensor.h
 * @brief Class to emulate magnetometer
 */

#ifndef MagSensor_H_
#define MagSensor_H_

#include <interface/log_output/loggable.hpp>

#include <Library/math/Quaternion.hpp>
#include <environment/local/local_environment.hpp>

#include "../Abstract/ComponentBase.h"
#include "../Abstract/SensorBase.h"

const size_t kMagDim = 3;  //!< Dimension of magnetometer

/**
 * @class MagSensor
 * @brief Class to emulate magnetometer
 */
class MagSensor : public ComponentBase, public SensorBase<kMagDim>, public ILoggable {
 public:
  /**
   * @fn MagSensor
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] sensor_base: Sensor base information
   * @param [in] sensor_id: Sensor ID
   * @param [in] q_b2c: Quaternion from body frame to component frame
   * @param [in] magnet: Geomagnetic environment
   */
  MagSensor(const int prescaler, ClockGenerator* clock_gen, SensorBase& sensor_base, const int sensor_id, const libra::Quaternion& q_b2c,
            const MagEnvironment* magnet);
  /**
   * @fn MagSensor
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] power_port: Power port
   * @param [in] sensor_base: Sensor base information
   * @param [in] sensor_id: Sensor ID
   * @param [in] q_b2c: Quaternion from body frame to component frame
   * @param [in] magnet: Geomagnetic environment
   */
  MagSensor(const int prescaler, ClockGenerator* clock_gen, PowerPort* power_port, SensorBase& sensor_base, const int sensor_id,
            const libra::Quaternion& q_b2c, const MagEnvironment* magnet);
  /**
   * @fn ~MagSensor
   * @brief Destructor
   */
  ~MagSensor();

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
   * @fn GetMagC
   * @brief Return observed magnetic field on the component frame
   */
  inline const libra::Vector<kMagDim>& GetMagC(void) const { return mag_c_; }

 protected:
  libra::Vector<kMagDim> mag_c_{0.0};            // observed magnetic field on the component frame [nT]
  int sensor_id_ = 0;                            //!< Sensor ID
  libra::Quaternion q_b2c_{0.0, 0.0, 0.0, 1.0};  //!< Quaternion from body frame to component frame

  const MagEnvironment* magnet_;  //!< Geomagnetic environment
};

#endif
