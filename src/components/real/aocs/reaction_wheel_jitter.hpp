/*
 * @file reaction_wheel_jitter.hpp
 * @brief Class to calculate RW high-frequency jitter effect
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_JITTER_HPP_
#define S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_JITTER_HPP_

#pragma once
#include <math_physics/math/quaternion.hpp>
#include <math_physics/math/vector.hpp>
#include <vector>

namespace s2e::components {

/*
 * @class ReactionWheelJitter
 * @brief Class to calculate RW high-frequency jitter effect
 */
class ReactionWheelJitter {
 public:
  /**
   * @fn ReactionWheelJitter
   * @brief Default Constructor
   */
  ReactionWheelJitter() {}
  /**
   * @fn ReactionWheelJitter
   * @brief Constructor
   * @param [in] radial_force_harmonics_coefficients: Coefficients for radial force harmonics
   * @param [in] radial_torque_harmonics_coefficients: Coefficients for radial torque harmonics
   * @param [in] update_interval_s: Jitter update interval [sec]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] structural_resonance_frequency_Hz: Frequency of structural resonance [Hz]
   * @param [in] damping_factor: Damping factor of structural resonance
   * @param [in] bandwidth: Bandwidth of structural resonance
   * @param [in] considers_structural_resonance: Flag to consider structural resonance
   */
  ReactionWheelJitter(std::vector<std::vector<double>> radial_force_harmonics_coefficients,
                      std::vector<std::vector<double>> radial_torque_harmonics_coefficients, const double update_interval_s,
                      const s2e::math::Quaternion quaternion_b2c, const double structural_resonance_frequency_Hz, const double damping_factor,
                      const double bandwidth, const bool considers_structural_resonance);
  /**
   * @fn ~ReactionWheelJitter
   * @brief Destructor
   */
  ~ReactionWheelJitter();

  /**
   * @fn CalcJitter
   * @brief Calculate Jitter
   * @param [in] angular_velocity_rad: Current angular velocity of RW [rad/s]
   */
  void CalcJitter(double angular_velocity_rad);

  /**
   * @fn GetJitterForce_b_N
   * @brief Return generated jitter force in the body fixed frame [N]
   */
  const s2e::math::Vector<3> GetJitterForce_b_N() const { return jitter_force_b_N_; }
  /**
   * @fn GetJitterTorque_b_Nm
   * @brief Return generated jitter torque in the body fixed frame [Nm]
   */
  const s2e::math::Vector<3> GetJitterTorque_b_Nm() const { return jitter_torque_b_Nm_; }
  /**
   * @fn GetJitterForce_c_N
   * @brief Return generated jitter force in the components frame [N]
   */
  const s2e::math::Vector<3> GetJitterForce_c_N() const {
    return considers_structural_resonance_ ? filtered_jitter_force_n_c_ : unfiltered_jitter_force_n_c_;
  }
  /**
   * @fn GetJitterTorque_c_Nm
   * @brief Return generated jitter torque in the component frame [Nm]
   */
  const s2e::math::Vector<3> GetJitterTorque_c_Nm() const {
    return considers_structural_resonance_ ? filtered_jitter_torque_n_c_ : unfiltered_jitter_torque_n_c_;
  }

 private:
  std::vector<std::vector<double>> radial_force_harmonics_coefficients_;   //!< Coefficients for radial force harmonics
  std::vector<std::vector<double>> radial_torque_harmonics_coefficients_;  //!< Coefficients for radial torque harmonics

  double update_interval_s_;         //!< Jitter update interval [sec]
  s2e::math::Quaternion quaternion_b2c_;  //!< Quaternion from body frame to component frame

  double structural_resonance_frequency_Hz_;          //!< Frequency of structural resonance [Hz]
  double structural_resonance_angular_frequency_Hz_;  //!< Angular Frequency of structural resonance
  double damping_factor_;                             //!< Damping factor of structural resonance
  double bandwidth_;                                  //!< Bandwidth of structural resonance
  bool considers_structural_resonance_;               //!< Flag to consider structural resonance

  // Jitter calculation variables
  std::vector<double> jitter_force_rotation_phase_;   //!< 2 * pi * h_i * Omega * t [rad]
  std::vector<double> jitter_torque_rotation_phase_;  //!< 2 * pi * h_i * Omega * t [rad]

  // Variables for solving difference equations in component frame
  s2e::math::Vector<3> unfiltered_jitter_force_n_c_{0.0};
  s2e::math::Vector<3> unfiltered_jitter_force_n_1_c_{0.0};
  s2e::math::Vector<3> unfiltered_jitter_force_n_2_c_{0.0};
  s2e::math::Vector<3> unfiltered_jitter_torque_n_c_{0.0};
  s2e::math::Vector<3> unfiltered_jitter_torque_n_1_c_{0.0};
  s2e::math::Vector<3> unfiltered_jitter_torque_n_2_c_{0.0};
  s2e::math::Vector<3> filtered_jitter_force_n_c_{0.0};
  s2e::math::Vector<3> filtered_jitter_force_n_1_c_{0.0};
  s2e::math::Vector<3> filtered_jitter_force_n_2_c_{0.0};
  s2e::math::Vector<3> filtered_jitter_torque_n_c_{0.0};
  s2e::math::Vector<3> filtered_jitter_torque_n_1_c_{0.0};
  s2e::math::Vector<3> filtered_jitter_torque_n_2_c_{0.0};
  double coefficients_[6];  //!< Coefficients of difference equation

  s2e::math::Vector<3> jitter_force_b_N_{0.0};    //!< Generated jitter force in the body frame [N]
  s2e::math::Vector<3> jitter_torque_b_Nm_{0.0};  //!< Generated jitter torque in the body frame [Nm]

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
   * @fn CalcCoefficients
   * @brief Calculation coefficients
   */
  void CalcCoefficients();
};

} // namespace s2e::components

#endif  // S2E_COMPONENTS_REAL_AOCS_REACTION_WHEEL_JITTER_HPP_
