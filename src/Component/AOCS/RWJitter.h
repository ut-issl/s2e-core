#pragma once
#include <vector>
#include <Library/math/Vector.hpp>
#include <Library/math/Quaternion.hpp>
class RWJitter
{
public:
  RWJitter(
    std::vector<std::vector<double>> radial_force_harmonics_coef,
    std::vector<std::vector<double>> radial_torque_harmonics_coef,
    const double jitter_update_interval,
    const libra::Quaternion q_b2c,
    const double structural_resonance_freq,
    const double damping_factor,
    const double bandwidth,
    const bool considers_structural_resonance
  );
  ~RWJitter();
  void CalcJitter(double angular_velocity_rad);
  const libra::Vector<3> GetJitterForceB() const { return jitter_force_b_; }
  const libra::Vector<3> GetJitterTorqueB() const { return jitter_torque_b_; }
  const libra::Vector<3> GetJitterForceC() const { return considers_structural_resonance_ ? filtered_jitter_force_n_c_ : unfiltered_jitter_force_n_c_; }
  const libra::Vector<3> GetJitterTorqueC() const { return considers_structural_resonance_ ? filtered_jitter_torque_n_c_ : unfiltered_jitter_torque_n_c_; }

private:
  std::vector<std::vector<double>> radial_force_harmonics_coef_;
  std::vector<std::vector<double>> radial_torque_harmonics_coef_;
  const double jitter_update_interval_;//sec
  libra::Quaternion q_b2c_;
  double structural_resonance_freq_;
  double structural_resonance_angular_freq_;
  double damping_factor_;
  double bandwidth_;
  bool considers_structural_resonance_;

  //Jitter calculation variables
  std::vector<double> jitter_force_rot_phase_;//2 * pi * h_i * Omega * t[rad]
  std::vector<double> jitter_torque_rot_phase_;//2 * pi * h_i * Omega * t[rad]

  //Variables for solving difference equations in compo frame
  libra::Vector<3> unfiltered_jitter_force_n_c_{ 0.0 }; 
  libra::Vector<3> unfiltered_jitter_force_n_1_c_{ 0.0 };
  libra::Vector<3> unfiltered_jitter_force_n_2_c_{ 0.0 };
  libra::Vector<3> unfiltered_jitter_torque_n_c_{ 0.0 }; 
  libra::Vector<3> unfiltered_jitter_torque_n_1_c_{ 0.0 };
  libra::Vector<3> unfiltered_jitter_torque_n_2_c_{ 0.0 };
  libra::Vector<3> filtered_jitter_force_n_c_{ 0.0 };
  libra::Vector<3> filtered_jitter_force_n_1_c_{ 0.0 };
  libra::Vector<3> filtered_jitter_force_n_2_c_{ 0.0 };
  libra::Vector<3> filtered_jitter_torque_n_c_{ 0.0 };
  libra::Vector<3> filtered_jitter_torque_n_1_c_{ 0.0 };
  libra::Vector<3> filtered_jitter_torque_n_2_c_{ 0.0 };
  double coef_[6]; //Coefficients of difference equation

  //Jitter force and torque in body frame
  libra::Vector<3> jitter_force_b_{ 0.0 };
  libra::Vector<3> jitter_torque_b_{ 0.0 };

  void AddStructuralResonance();
  void ShiftTimeStep();
  void CalcCoef();
};
