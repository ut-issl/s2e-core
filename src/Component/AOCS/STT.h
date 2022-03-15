#pragma once

#ifndef __STT_H__
#define __STT_H__

#include <Dynamics/Attitude/Attitude.h>
#include <Environment/Local/LocalEnvironment.h>
#include <Interface/LogOutput/ILoggable.h>

#include <Library/math/NormalRand.hpp>
#include <Library/math/Quaternion.hpp>
#include <Library/math/Ran1.hpp>
#include <Library/math/Vector.hpp>
#include <vector>

#include "../Abstract/ComponentBase.h"
#include "Dynamics/Dynamics.h"

class STT : public ComponentBase, public ILoggable {
 public:
  /**
   * @brief Constructor
   * @param[in] prescaler and clock_gen : arguments for ComponentBase:
   * @param[in] id ID for log output
   * @param[in] q_b2c Quaternion from body frame to component frame
   * @param[in] sigma_ortho Standard deviation for random noise in orthogonal
   * direction of sight [rad]
   * @param[in] sigma_sight Standard deviation for random noise in sight
   * direction[rad]
   * @param[in] step_time Step time [sec]
   * @param[in] output_delay output delay. unit: step_time,[0-MAX_DELAY]
   * @param[in] output_interval output intercal unit:step_time
   * @param[in] forbidden_angles forbidden angles for each body [rad]．
   * @param[in] capture_rate Angular rate limit to get correct attitude[rad/s]
   */
  STT(const int prescaler, ClockGenerator* clock_gen, const int id,
      const libra::Quaternion& q_b2c, const double sigma_ortho,
      const double sigma_sight, const double step_time,
      const unsigned int output_delay, const unsigned int output_interval,
      const double sun_forbidden_angle, const double earth_forbidden_angle,
      const double moon_forbidden_angle, const double capture_rate,
      const Dynamics* dynamics, const LocalEnvironment* local_env);
  STT(const int prescaler, ClockGenerator* clock_gen, PowerPort* power_port,
      const int id, const libra::Quaternion& q_b2c, const double sigma_ortho,
      const double sigma_sight, const double step_time,
      const unsigned int output_delay, const unsigned int output_interval,
      const double sun_forbidden_angle, const double earth_forbidden_angle,
      const double moon_forbidden_angle, const double capture_rate,
      const Dynamics* dynamics, const LocalEnvironment* local_env);

  // ComponentBase override functions
  void MainRoutine(int count) override;
  // ILogabble override functions
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // Getter
  inline const libra::Quaternion GetObsQuaternion() const {
    return q_stt_i2c_;
  };
  inline const bool GetErrorFlag() const { return error_flag_; }

 protected:
  // STT general parameters
  const int id_;
  libra::Quaternion q_b2c_;  // Quaternion from body frame to component frame
  libra::Quaternion q_stt_i2c_ = {0.0, 0.0, 0.0,
                                  1.0};  // STT observed quaternion
  libra::Vector<3> sight_;  // sight direction vector at component frame
  libra::Vector<3>
      ortho1_;  // the first orthogonal direction of sight at component frame
  libra::Vector<3>
      ortho2_;  // the secont orthogonal direction of sight at component frame

  // Noise parameters
  libra::Ran1 rot_;  // Randomize object for orthogonal direction
  libra::NormalRand
      n_ortho_;  // Random noise for orthogonal direction of sight [rad]
  libra::NormalRand n_sight_;  // Random noise for sight direction [rad]

  // Delay emulation parameters
  int MAX_DELAY;
  std::vector<Quaternion> q_buffer_;  // buffer for delay
  int pos_;                           // buffer position
  double step_time_;                  // sec
  unsigned int output_delay_;  // output delay. unit: step_time,[0-MAX_DELAY]
  unsigned int
      output_interval_;  // output interval. unit: step_time,[0-MAX_DELAY]
  std::size_t count_;    // output update counter

  // observation error parameters
  bool error_flag_;               // true: Error , false: No error
  double sun_forbidden_angle_;    // rad
  double earth_forbidden_angle_;  // rad
  double moon_forbidden_angle_;   // rad
  double capture_rate_;  // Angular rate limit to get correct attitude[rad/s]

  // Observed variables
  const Dynamics* dynamics_;
  const LocalEnvironment* local_env_;

  // Internal functions
  void update(const LocalCelestialInformation* local_celes_info,
              const Attitude* attinfo);  // update delay buffer
  libra::Quaternion measure(
      const LocalCelestialInformation* local_celes_info,
      const Attitude* attinfo);  // Calc measured quaternion

  void AllJudgement(const LocalCelestialInformation* local_celes_info,
                    const Attitude* attinfo);
  int SunJudgement(const libra::Vector<3>& sun_b);  // Judge sun forbidden angle
  int EarthJudgement(
      const libra::Vector<3>& earth_b);  // Judge earth forbidden angle
  int MoonJudgement(
      const libra::Vector<3>& moon_b);  // Judge moon forbidden angle
  int CaptureRateJudgement(
      const libra::Vector<3>& omega_b);  // Judge limit angular velocity
  double CalAngleVect_rad(const libra::Vector<3>& vect1,
                          const libra::Vector<3>& vect2);

  void Initialize();
};

#endif
