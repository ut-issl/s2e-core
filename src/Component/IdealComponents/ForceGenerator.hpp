#pragma once

#include <Component/Abstract/ComponentBase.h>
#include <Dynamics/Dynamics.h>
#include <Interface/LogOutput/Logger.h>

#include <Library/math/NormalRand.hpp>
#include <Library/math/Vector.hpp>

class ForceGenerator : public ComponentBase, public ILoggable {
 public:
  ForceGenerator(const int prescaler, ClockGenerator* clock_gen, const double magnitude_error_standard_deviation_N,
                 const double direction_error_standard_deviation_rad, const Dynamics* dynamics);
  ~ForceGenerator();

  // ComponentBase override function
  void MainRoutine(int count);
  void PowerOffRoutine();

  // ILogabble override function
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // Getter
  inline const Vector<3> GetGeneratedForce_b_N() const { return generated_force_b_N_; };
  inline const Vector<3> GetGeneratedForce_i_N() const { return generated_force_i_N_; };
  inline const Vector<3> GetGeneratedForce_rtn_N() const { return generated_force_rtn_N_; };

  // Setter
  inline void SetForce_b_N(const libra::Vector<3> force_b_N) { ordered_force_b_N_ = force_b_N; };
  void SetForce_i_N(const libra::Vector<3> force_i_N);
  void SetForce_rtn_N(const libra::Vector<3> force_rtn_N);

 protected:
  libra::Vector<3> ordered_force_b_N_{0.0};
  libra::Vector<3> generated_force_b_N_{0.0};
  libra::Vector<3> generated_force_i_N_{0.0};
  libra::Vector<3> generated_force_rtn_N_{0.0};

  // Noise
  libra::NormalRand magnitude_noise_;
  libra::NormalRand direction_noise_;
  double direction_error_standard_deviation_rad_;
  libra::Quaternion GenerateDirectionNoiseQuaternion(libra::Vector<3> true_direction, const double error_standard_deviation_rad);

  const Dynamics* dynamics_;
};
