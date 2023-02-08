/*
 * @file reaction_wheel_jitter.hpp
 * @brief Class to calculate RW high-frequency jitter effect
 */

#ifndef S2E_COMPONENTS_AOCS_REACTION_WHEEL_JITTER_H_
#define S2E_COMPONENTS_AOCS_REACTION_WHEEL_JITTER_H_

#pragma once
#include <library/math/quaternion.hpp>
#include <library/math/vector.hpp>
#include <vector>

/*
 * @class RWJitter
 * @brief Class to calculate RW high-frequency jitter effect
 */
class RWJitter {
 public:
  /**
   * @fn RWJitter
   * @brief Constructor
   * @param [in] radial_force_harmonics_coef: Coefficients for radial force harmonics
   * @param [in] radial_torque_harmonics_coef: Coefficients for radial torque harmonics
   * @param [in] jitter_update_interval: Jitter update interval [sec]
   * @param [in] q_b2c: Quaternion from body frame to component frame
   * @param [in] structural_resonance_freq: Frequency of structural resonance [Hz]
   * @param [in] damping_factor: Damping factor of structural resonance
   * @param [in] bandwidth: Bandwidth of structural resonance
   * @param [in] considers_structural_resonance: Flag to consider structural resonance
   */
  RWJitter(std::vector<std::vector<double>> radial_force_harmonics_coef, std::vector<std::vector<double>> radial_torque_harmonics_coef,
           const double jitter_update_interval, const libra::Quaternion q_b2c, const double structural_resonance_freq, const double damping_factor,
           const double bandwidth, const bool considers_structural_resonance);
  /**
   * @fn ~RWJitter
   * @brief Destructor
   */
  ~RWJitter();

  /**
   * @fn CalcJitter
   * @brief Calculate Jitter
   * @param [in] angular_velocity_rad: Current angular velocity of RW [rad/s]
   */
  void CalcJitter(double angular_velocity_rad);

  /**
   * @fn GetJitterForceB
   * @brief Return generated jitter force in the body fixed frame [N]
   */
  const libra::Vector<3> GetJitterForceB() const { return jitter_force_b_; }
  /**
   * @fn GetJitterTorqueB
   * @brief Return generated jitter torque in the body fixed frame [Nm]
   */
  const libra::Vector<3> GetJitterTorqueB() const { return jitter_torque_b_; }
  /**
   * @fn GetJitterForceC
   * @brief Return generated jitter force in the components frame [N]
   */
  const libra::Vector<3> GetJitterForceC() const {
    return considers_structural_resonance_ ? filtered_jitter_force_n_c_ : unfiltered_jitter_force_n_c_;
  }
  /**
   * @fn GetJitterTorqueC
   * @brief Return generated jitter torque in the component frame [Nm]
   */
  const libra::Vector<3> GetJitterTorqueC() const {
    return considers_structural_resonance_ ? filtered_jitter_torque_n_c_ : unfiltered_jitter_torque_n_c_;
  }

 private:
  std::vector<std::vector<double>> radial_force_harmonics_coef_;   //!< Coefficients for radial force harmonics
  std::vector<std::vector<double>> radial_torque_harmonics_coef_;  //!< Coefficients for radial torque harmonics

  const double jitter_update_interval_;  //!< Jitter update interval [sec]
  libra::Quaternion q_b2c_;              //!< Quaternion from body frame to component frame

  double structural_resonance_freq_;          //!< Frequency of structural resonance [Hz]
  double structural_resonance_angular_freq_;  //!< Angular Frequency of structural resonance
  double damping_factor_;                     //!< Damping factor of structural resonance
  double bandwidth_;                          //!< Bandwidth of structural resonance
  bool considers_structural_resonance_;       //!< Flag to consider structural resonance

  // Jitter calculation variables
  std::vector<double> jitter_force_rot_phase_;   //!< 2 * pi * h_i * Omega * t [rad]
  std::vector<double> jitter_torque_rot_phase_;  //!< 2 * pi * h_i * Omega * t [rad]

  // Variables for solving difference equations in compo frame
  libra::Vector<3> unfiltered_jitter_force_n_c_{0.0};
  libra::Vector<3> unfiltered_jitter_force_n_1_c_{0.0};
  libra::Vector<3> unfiltered_jitter_force_n_2_c_{0.0};
  libra::Vector<3> unfiltered_jitter_torque_n_c_{0.0};
  libra::Vector<3> unfiltered_jitter_torque_n_1_c_{0.0};
  libra::Vector<3> unfiltered_jitter_torque_n_2_c_{0.0};
  libra::Vector<3> filtered_jitter_force_n_c_{0.0};
  libra::Vector<3> filtered_jitter_force_n_1_c_{0.0};
  libra::Vector<3> filtered_jitter_force_n_2_c_{0.0};
  libra::Vector<3> filtered_jitter_torque_n_c_{0.0};
  libra::Vector<3> filtered_jitter_torque_n_1_c_{0.0};
  libra::Vector<3> filtered_jitter_torque_n_2_c_{0.0};
  double coef_[6];  //!< Coefficients of difference equation

  libra::Vector<3> jitter_force_b_{0.0};   //!< Generated jitter force in the body frame [N]
  libra::Vector<3> jitter_torque_b_{0.0};  //!< Generated jitter torque in the body frame [Nm]

  /**
   * @fn AddStructuralResonance
   * @brief Add structural resonance effect
   */
  void AddStructuralResonance();
  /**
   * @fn ShiftTimeStep
   * @brief Shift time step
   */
  void ShiftTimeStep();
  /**
   * @fn CalcCoef
   * @brief Calculation coefficients
   */
  void CalcCoef();
};

#endif  // S2E_COMPONENTS_AOCS_REACTION_WHEEL_JITTER_H_
