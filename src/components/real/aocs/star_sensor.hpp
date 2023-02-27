/*
 * @file star_sensor.hpp
 * @brief Class to emulate star tracker
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_STAR_SENSOR_HPP_
#define S2E_COMPONENTS_REAL_AOCS_STAR_SENSOR_HPP_

#include <dynamics/attitude/attitude.hpp>
#include <environment/local/local_environment.hpp>
#include <library/logger/loggable.hpp>
#include <library/math/quaternion.hpp>
#include <library/math/vector.hpp>
#include <library/randomization/minimal_standard_linear_congruential_generator_with_shuffle.hpp>
#include <library/randomization/normal_randomization.hpp>
#include <vector>

#include "../../base/component.hpp"
#include "dynamics/dynamics.hpp"

/*
 * @class STT
 * @brief Class to emulate star tracker
 */
class STT : public Component, public ILoggable {
 public:
  /**
   * @fn STT
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] sigma_ortho: Standard deviation for random noise in orthogonal direction of sight [rad]
   * @param [in] sigma_sight: Standard deviation for random noise in sight direction[rad]
   * @param [in] step_time: Step time for delay calculation [sec]
   * @param [in] output_delay: Output delay [0, MAX_DELAY] [step_sec]
   * @param [in] output_interval: Output interval [step_sec]
   * @param [in] sun_forbidden_angle: Sun forbidden angle [rad]
   * @param [in] earth_forbidden_angle: Earth forbidden angle [rad]
   * @param [in] moon_forbidden_angle: Moon forbidden angle [rad]
   * @param [in] capture_rate: Angular rate limit to get correct attitude [rad/s]
   * @param [in] dynamics: Dynamics information
   * @param [in] local_env: Local environment information
   */
  STT(const int prescaler, ClockGenerator* clock_generator, const int component_id, const libra::Quaternion& quaternion_b2c, const double sigma_ortho,
      const double sigma_sight, const double step_time, const unsigned int output_delay, const unsigned int output_interval,
      const double sun_forbidden_angle, const double earth_forbidden_angle, const double moon_forbidden_angle, const double capture_rate,
      const Dynamics* dynamics, const LocalEnvironment* local_env);
  /**
   * @fn STT
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] component_id: Sensor ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] sigma_ortho: Standard deviation for random noise in orthogonal direction of sight [rad]
   * @param [in] sigma_sight: Standard deviation for random noise in sight direction[rad]
   * @param [in] step_time: Step time for delay calculation [sec]
   * @param [in] output_delay: Output delay [0, MAX_DELAY] [step_sec]
   * @param [in] output_interval: Output interval [step_sec]
   * @param [in] sun_forbidden_angle: Sun forbidden angle [rad]
   * @param [in] earth_forbidden_angle: Earth forbidden angle [rad]
   * @param [in] moon_forbidden_angle: Moon forbidden angle [rad]
   * @param [in] capture_rate: Angular rate limit to get correct attitude [rad/s]
   * @param [in] dynamics: Dynamics information
   * @param [in] local_env: Local environment information
   */
  STT(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id, const libra::Quaternion& quaternion_b2c,
      const double sigma_ortho, const double sigma_sight, const double step_time, const unsigned int output_delay, const unsigned int output_interval,
      const double sun_forbidden_angle, const double earth_forbidden_angle, const double moon_forbidden_angle, const double capture_rate,
      const Dynamics* dynamics, const LocalEnvironment* local_env);

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
   * @fn GetObsQuaternion
   * @brief Return observed quaternion from the inertial frame to the component frame
   */
  inline const libra::Quaternion GetObsQuaternion() const { return q_stt_i2c_; };
  /**
   * @fn GetErrorFlag
   * @brief Return error flag
   */
  inline bool GetErrorFlag() const { return error_flag_; }

 protected:
  // STT general parameters
  const int component_id_;                              //!< Sensor ID
  libra::Quaternion quaternion_b2c_;                    //!< Quaternion from body frame to component frame
  libra::Quaternion q_stt_i2c_ = {0.0, 0.0, 0.0, 1.0};  //!< STT observed quaternion
  libra::Vector<3> sight_;                              //!< Sight direction vector at component frame
  libra::Vector<3> ortho1_;                             //!< The first orthogonal direction of sight at component frame
  libra::Vector<3> ortho2_;                             //!< The second orthogonal direction of sight at component frame

  // Noise parameters
  libra::MinimalStandardLcgWithShuffle rot_;  //!< Randomize object for orthogonal direction
  libra::NormalRand n_ortho_;                 //!< Random noise for orthogonal direction of sight [rad]
  libra::NormalRand n_sight_;                 //!< Random noise for sight direction [rad]

  // Delay emulation parameters
  int MAX_DELAY;                      //!< Max delay
  std::vector<Quaternion> q_buffer_;  //!< Buffer of quaternion for delay emulation
  int pos_;                           //!< Buffer position
  double step_time_;                  //!< Step time for delay calculation [sec]
  unsigned int output_delay_;         //!< Output delay [0, MAX_DELAY] [step_sec]
  unsigned int output_interval_;      //!< Output interval [step_sec]
  std::size_t count_;                 //!< Output update counter

  // observation error parameters
  bool error_flag_;               //!< Error flag. true: Error, false: No error
  double sun_forbidden_angle_;    //!< Sun forbidden angle [rad]
  double earth_forbidden_angle_;  //!< Earth forbidden angle [rad]
  double moon_forbidden_angle_;   //!< Moon forbidden angle [rad]
  double capture_rate_;           //!< Angular rate limit to get correct attitude [rad/s]

  // Observed variables
  const Dynamics* dynamics_;           //!< Dynamics information
  const LocalEnvironment* local_env_;  //!< Local environment information

  // Internal functions
  /**
   * @fn update
   * @brief Update delay buffer
   * @param [in] local_celes_info: Local celestial information
   * @param [in] attinfo: Attitude information
   */
  void update(const LocalCelestialInformation* local_celes_info, const Attitude* attinfo);
  /**
   * @fn measure
   * @brief Calculate measured quaternion
   * @param [in] local_celes_info: Local celestial information
   * @param [in] attinfo: Attitude information
   */
  libra::Quaternion measure(const LocalCelestialInformation* local_celes_info, const Attitude* attinfo);

  /**
   * @fn AllJudgement
   * @brief Calculate all error judgement
   * @param [in] local_celes_info: Local celestial information
   * @param [in] attinfo: Attitude information
   */
  void AllJudgement(const LocalCelestialInformation* local_celes_info, const Attitude* attinfo);
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
   * @param [in] omega_b: Angular velocity of spacecraft in the body fixed frame
   * @return 1: violated, 0: not violated
   */
  int CaptureRateJudgement(const libra::Vector<3>& omega_b);
  /**
   * @fn CalAngleVect_rad
   * @brief Calculate angle between two vectors
   * @param [in] vect1: First vector
   * @param [in] vect2: Second vector
   * @return Angle between two vectors [rad]
   */
  double CalAngleVect_rad(const libra::Vector<3>& vect1, const libra::Vector<3>& vect2);

  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize();
};

#endif  // S2E_COMPONENTS_REAL_AOCS_STAR_SENSOR_HPP_
