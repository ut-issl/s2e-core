/*
 * @file sun_sensor.hpp
 * @brief Class to emulate sun sensor
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_SUN_SENSOR_HPP_
#define S2E_COMPONENTS_REAL_AOCS_SUN_SENSOR_HPP_

#include <environment/local/local_celestial_information.hpp>
#include <environment/local/solar_radiation_pressure_environment.hpp>
#include <library/logger/loggable.hpp>
#include <library/math/quaternion.hpp>
#include <library/math/vector.hpp>
#include <library/randomization/normal_randomization.hpp>

#include "../../base/component.hpp"

/*
 * @class SunSensor
 * @brief Class to emulate sun sensor
 */
class SunSensor : public Component, public ILoggable {
 public:
  /**
   * @fn SunSensor
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] detectable_angle_rad: Detectable angle threshold [rad]
   * @param [in] normal_random_standard_deviation_c_Am2: Standard deviation of normal random noise in the component frame [rad]
   * @param [in] nr_bias_stddev_c: Standard deviation of normal random noise for bias in the component frame [rad]
   * @param [in] intensity_lower_threshold_percent: Solar intensity lower threshold [%]
   * @param [in] srp: Solar Radiation Pressure environment
   * @param [in] local_celestial_information: Local celestial information
   */
  SunSensor(const int prescaler, ClockGenerator* clock_generator, const int component_id, const libra::Quaternion& quaternion_b2c,
            const double detectable_angle_rad, const double normal_random_standard_deviation_c_Am2, const double nr_bias_stddev_c,
            const double intensity_lower_threshold_percent, const SolarRadiationPressureEnvironment* srp,
            const LocalCelestialInformation* local_celestial_information);
  /**
   * @fn SunSensor
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] component_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] detectable_angle_rad: Detectable angle threshold [rad]
   * @param [in] normal_random_standard_deviation_c_Am2: Standard deviation of normal random noise in the component frame [rad]
   * @param [in] nr_bias_stddev_c: Standard deviation of normal random noise for bias in the component frame [rad]
   * @param [in] intensity_lower_threshold_percent: Solar intensity lower threshold [%]
   * @param [in] srp: Solar Radiation Pressure environment
   * @param [in] local_celestial_information: Local celestial information
   */
  SunSensor(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
            const libra::Quaternion& quaternion_b2c, const double detectable_angle_rad, const double normal_random_standard_deviation_c_Am2,
            const double nr_bias_stddev_c, const double intensity_lower_threshold_percent, const SolarRadiationPressureEnvironment* srp,
            const LocalCelestialInformation* local_celestial_information);

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

  // Getter
  inline bool GetSunDetectedFlag() const { return sun_detected_flag_; };
  inline const libra::Vector<3> GetMeasuredSun_c() const { return measured_sun_direction_c_; };
  inline const libra::Vector<3> GetMeasuredSun_b() const { return quaternion_b2c_.Conjugate().FrameConversion(measured_sun_direction_c_); };
  inline double GetSunAngleAlpha() const { return alpha_rad_; };
  inline double GetSunAngleBeta() const { return beta_rad_; };
  inline double GetSolarIlluminance() const { return solar_illuminance_W_m2_; };

 protected:
  const int component_id_;                    //!< Sensor ID
  libra::Quaternion quaternion_b2c_;          //!< Quaternion from body frame to component frame (Z-axis of the component is sight direction)
  double intensity_lower_threshold_percent_;  //!< If the light intensity becomes smaller than this, it becomes impossible to get the sun direction

  libra::Vector<3> sun_direction_true_c_{0.0};      //!< True value of sun vector in the component frame
  libra::Vector<3> measured_sun_direction_c_{0.0};  //!< Measured sun vector in the component frame

  double alpha_rad_ = 0.0;               //!< Angle between Z-axis and the sun direction projected on XZ plane [rad]
  double beta_rad_ = 0.0;                //!< Angle between Z-axis and the sun direction projected on YZ plane [rad]
  double solar_illuminance_W_m2_ = 0.0;  //!< The energy of sunlight per unit area, taking into account the angle to the sun [W/m^2]
  double detectable_angle_rad_;          //!< half angle (>0) [rad]
  bool sun_detected_flag_ = false;       //!< Sun detected flag
  // Noise parameters
  libra::NormalRand random_noise_alpha_;  //!< Normal random for alpha angle
  libra::NormalRand random_noise_beta_;   //!< Normal random for beta angle
  double bias_noise_alpha_rad_ = 0.0;     //!< Constant bias for alpha angle (Value is calculated by random number generator)
  double bias_noise_beta_rad_ = 0.0;      //!< Constant bias for beta angle (Value is calculated by random number generator)

  // Measured variables
  const SolarRadiationPressureEnvironment* srp_;                  //!< Solar Radiation Pressure environment
  const LocalCelestialInformation* local_celestial_information_;  //!< Local celestial information

  // functions
  /**
   * @fn SunDetectionJudgement
   * @brief Judge sun is detected or not
   */
  void SunDetectionJudgement();
  /**
   * @fn measure
   * @brief Calculate observed sun angle
   */
  void measure();
  /**
   * @fn TanRange
   * @brief Clip angle as tangent range
   * @param [in] x: Input angle
   * @return Clipped value
   */
  double TanRange(double x);
  /**
   * @fn TanRange
   * @brief Clip angle as tangent range
   * @param [in] normal_random_standard_deviation_c_Am2: Standard deviation of normal random noise in the component frame [rad]
   * @param [in] nr_bias_stddev_c: Standard deviation of normal random noise for bias in the component frame [rad]
   */
  void Initialize(const double normal_random_standard_deviation_c_Am2, const double nr_bias_stddev_c);
  /**
   * @fn CalcSolarIlluminance
   * @brief Calculate solar illuminance on the sun sensor surface
   */
  void CalcSolarIlluminance();
};

#endif  // S2E_COMPONENTS_REAL_AOCS_SUN_SENSOR_HPP_
