/*
 * @file star_sensor.hpp
 * @brief Class to emulate star tracker
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_STAR_SENSOR_HPP_
#define S2E_COMPONENTS_REAL_AOCS_STAR_SENSOR_HPP_

#include <dynamics/attitude/attitude.hpp>
#include <environment/local/local_environment.hpp>
#include <math_physics/math/quaternion.hpp>
#include <math_physics/math/vector.hpp>
#include <math_physics/randomization/minimal_standard_linear_congruential_generator_with_shuffle.hpp>
#include <math_physics/randomization/normal_randomization.hpp>
#include <logger/loggable.hpp>
#include <vector>

#include "../../base/component.hpp"
#include "dynamics/dynamics.hpp"

/*
 * @class StarSensor
 * @brief Class to emulate star tracker
 */
class StarSensor : public Component, public ILoggable {
 public:
  /**
   * @fn StarSensor
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] standard_deviation_orthogonal_direction: Standard deviation for random noise in orthogonal direction of sight [rad]
   * @param [in] standard_deviation_sight_direction: Standard deviation for random noise in sight direction[rad]
   * @param [in] step_time_s: Step time for delay calculation [sec]
   * @param [in] output_delay: Output delay [0, max_delay_] [step_sec]
   * @param [in] output_interval: Output interval [step_sec]
   * @param [in] sun_forbidden_angle_rad: Sun forbidden angle [rad]
   * @param [in] earth_forbidden_angle_rad: Earth forbidden angle [rad]
   * @param [in] moon_forbidden_angle_rad: Moon forbidden angle [rad]
   * @param [in] capture_rate_limit_rad_s: Angular rate limit to get correct attitude [rad/s]
   * @param [in] dynamics: Dynamics information
   * @param [in] local_environment: Local environment information
   */
  StarSensor(const int prescaler, ClockGenerator* clock_generator, const int component_id, const libra::Quaternion& quaternion_b2c,
             const double standard_deviation_orthogonal_direction, const double standard_deviation_sight_direction, const double step_time_s,
             const unsigned int output_delay, const unsigned int output_interval, const double sun_forbidden_angle_rad,
             const double earth_forbidden_angle_rad, const double moon_forbidden_angle_rad, const double capture_rate_limit_rad_s,
             const Dynamics* dynamics, const LocalEnvironment* local_environment);
  /**
   * @fn StarSensor
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] component_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] standard_deviation_orthogonal_direction: Standard deviation for random noise in orthogonal direction of sight [rad]
   * @param [in] standard_deviation_sight_direction: Standard deviation for random noise in sight direction[rad]
   * @param [in] step_time_s: Step time for delay calculation [sec]
   * @param [in] output_delay: Output delay [0, max_delay_] [step_sec]
   * @param [in] output_interval: Output interval [step_sec]
   * @param [in] sun_forbidden_angle_rad: Sun forbidden angle [rad]
   * @param [in] earth_forbidden_angle_rad: Earth forbidden angle [rad]
   * @param [in] moon_forbidden_angle_rad: Moon forbidden angle [rad]
   * @param [in] capture_rate_limit_rad_s: Angular rate limit to get correct attitude [rad/s]
   * @param [in] dynamics: Dynamics information
   * @param [in] local_environment: Local environment information
   */
  StarSensor(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
             const libra::Quaternion& quaternion_b2c, const double standard_deviation_orthogonal_direction,
             const double standard_deviation_sight_direction, const double step_time_s, const unsigned int output_delay,
             const unsigned int output_interval, const double sun_forbidden_angle_rad, const double earth_forbidden_angle_rad,
             const double moon_forbidden_angle_rad, const double capture_rate_limit_rad_s, const Dynamics* dynamics,
             const LocalEnvironment* local_environment);

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
   * @fn GetMeasuredQuaternion_i2c
   * @brief Return observed quaternion from the inertial frame to the component frame
   */
  inline const libra::Quaternion GetMeasuredQuaternion_i2c() const { return measured_quaternion_i2c_; };
  /**
   * @fn GetErrorFlag
   * @brief Return error flag
   */
  inline bool GetErrorFlag() const { return error_flag_; }

 protected:
  // StarSensor general parameters
  const int component_id_;                                            //!< Sensor ID
  libra::Quaternion quaternion_b2c_;                                  //!< Quaternion from body frame to component frame
  libra::Quaternion measured_quaternion_i2c_ = {0.0, 0.0, 0.0, 1.0};  //!< StarSensor observed quaternion
  libra::Vector<3> sight_direction_c_;                                //!< Sight direction vector at component frame
  libra::Vector<3> first_orthogonal_direction_c;                      //!< The first orthogonal direction of sight at component frame
  libra::Vector<3> second_orthogonal_direction_c;                     //!< The second orthogonal direction of sight at component frame

  // Noise parameters
  libra::MinimalStandardLcgWithShuffle rotation_noise_;  //!< Randomize object for orthogonal direction
  libra::NormalRand orthogonal_direction_noise_;         //!< Random noise for orthogonal direction of sight
  libra::NormalRand sight_direction_noise_;              //!< Random noise for sight direction

  // Delay emulation parameters
  int max_delay_;                                //!< Max delay
  std::vector<libra::Quaternion> delay_buffer_;  //!< Buffer of quaternion for delay emulation
  int buffer_position_;                          //!< Buffer position
  double step_time_s_;                           //!< Step time for delay calculation [sec]
  unsigned int output_delay_;                    //!< Output delay [0, max_delay_] [step_sec]
  unsigned int output_interval_;                 //!< Output interval [step_sec]
  std::size_t update_count_;                     //!< Output update counter

  // observation error parameters
  bool error_flag_;                   //!< Error flag. true: Error, false: No error
  double sun_forbidden_angle_rad_;    //!< Sun forbidden angle [rad]
  double earth_forbidden_angle_rad_;  //!< Earth forbidden angle [rad]
  double moon_forbidden_angle_rad_;   //!< Moon forbidden angle [rad]
  double capture_rate_limit_rad_s_;   //!< Angular rate limit to get correct attitude [rad/s]

  // Observed variables
  const Dynamics* dynamics_;                   //!< Dynamics information
  const LocalEnvironment* local_environment_;  //!< Local environment information

  // Internal functions
  /**
   * @fn update
   * @brief Update delay buffer
   * @param [in] local_celestial_information: Local celestial information
   * @param [in] attitude: Attitude information
   */
  void update(const LocalCelestialInformation* local_celestial_information, const Attitude* attitude);
  /**
   * @fn Measure
   * @brief Calculate measured quaternion
   * @param [in] local_celestial_information: Local celestial information
   * @param [in] attitude: Attitude information
   */
  libra::Quaternion Measure(const LocalCelestialInformation* local_celestial_information, const Attitude* attitude);

  /**
   * @fn AllJudgement
   * @brief Calculate all error judgement
   * @param [in] local_celestial_information: Local celestial information
   * @param [in] attitude: Attitude information
   */
  void AllJudgement(const LocalCelestialInformation* local_celestial_information, const Attitude* attitude);
  /**
   * @fn SunJudgement
   * @brief Judge violation of sun forbidden angle
   * @param [in] sun_b: Sun direction vector in the body fixed frame
   * @return 1: violated, 0: not violated
   */
  int SunJudgement(const libra::Vector<3>& sun_b);
  /**
   * @fn EarthJudgement
   * @brief Judge violation of earth forbidden angle
   * @param [in] earth_b: Earth direction vector in the body fixed frame
   * @return 1: violated, 0: not violated
   */
  int EarthJudgement(const libra::Vector<3>& earth_b);
  /**
   * @fn MoonJudgement
   * @brief Judge violation of moon forbidden angle
   * @param [in] moon_b: Moon direction vector in the body fixed frame
   * @return 1: violated, 0: not violated
   */
  int MoonJudgement(const libra::Vector<3>& moon_b);
  /**
   * @fn CaptureRateJudgement
   * @brief Judge violation of angular velocity limit
   * @param [in] omega_b_rad_s: Angular velocity of spacecraft in the body fixed frame
   * @return 1: violated, 0: not violated
   */
  int CaptureRateJudgement(const libra::Vector<3>& omega_b_rad_s);
  /**
   * @fn CalAngleVector_rad
   * @brief Calculate angle between two vectors
   * @param [in] vector1: First vector
   * @param [in] vector2: Second vector
   * @return Angle between two vectors [rad]
   */
  double CalAngleVector_rad(const libra::Vector<3>& vector1, const libra::Vector<3>& vector2);

  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize();
};

/**
 * @fn InitStarSensor
 * @brief Initialize functions for StarSensor without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] dynamics: Dynamics information
 * @param [in] local_environment: Local environment information
 */
StarSensor InitStarSensor(ClockGenerator* clock_generator, int sensor_id, const std::string file_name, double component_step_time_s,
                          const Dynamics* dynamics, const LocalEnvironment* local_environment);
/**
 * @fn InitStarSensor
 * @brief Initialize functions for StarSensor with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] sensor_id: Sensor ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] dynamics: Dynamics information
 * @param [in] local_environment: Local environment information
 */
StarSensor InitStarSensor(ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const std::string file_name,
                          double component_step_time_s, const Dynamics* dynamics, const LocalEnvironment* local_environment);

#endif  // S2E_COMPONENTS_REAL_AOCS_STAR_SENSOR_HPP_
