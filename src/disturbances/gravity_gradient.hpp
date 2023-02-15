/**
 * @file gravity_gradient.hpp
 * @brief Class to calculate the gravity gradient torque
 */

#ifndef S2E_DISTURBANCES_GRAVITY_GRADIENT_HPP_
#define S2E_DISTURBANCES_GRAVITY_GRADIENT_HPP_

#include "../interface/log_output/loggable.hpp"
#include "../library/math/matrix.hpp"
#include "../library/math/matrix_vector.hpp"
#include "../library/math/vector.hpp"
#include "simple_disturbance.hpp"

/**
 * @class GravityGradient
 * @brief Class to calculate the gravity gradient torque
 */
class GravityGradient : public SimpleDisturbance {
 public:
  /**
   * @fn GeoPotential
   * @brief Default Constructor
   * @param [in] is_calculation_enabled: Calculation flag
   * @note mu is automatically set as earth's gravitational constant
   */
  GravityGradient(const bool is_calculation_enabled = true);
  /**
   * @fn GeoPotential
   * @brief Constructor
   * @param [in] mu_m3_s2: Gravitational constant [m3/s2]
   * @param [in] is_calculation_enabled: Calculation flag
   */
  GravityGradient(const double mu_m3_s2, const bool is_calculation_enabled = true);

  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
   * @param [in] local_environment: Local environment information
   * @param [in] dynamics: Dynamics information
   */
  virtual void Update(const LocalEnvironment& local_environment, const Dynamics& dynamics);

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

 private:
  double mu_m3_s2_;  //!< Gravitational constant [m3/s2]

  /**
   * @fn CalcTorque
   * @brief Calculate gravity gradient torque
   * @param [in] r_b_m: Position vector of the earth at body frame [m]
   * @param [in] I_b_kgm2: Inertia Tensor at body frame [kg*m^2]
   * @return Calculated torque at body frame [Nm]
   */
  libra::Vector<3> CalcTorque(const libra::Vector<3> r_b_m, const libra::Matrix<3, 3> I_b_kgm2);
};

#endif  // S2E_DISTURBANCES_GRAVITY_GRADIENT_HPP_
