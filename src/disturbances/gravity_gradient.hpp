/**
 * @file gravity_gradient.hpp
 * @brief Class to calculate the gravity gradient torque
 */

#ifndef S2E_DISTURBANCES_GRAVITY_GRADIENT_H_
#define S2E_DISTURBANCES_GRAVITY_GRADIENT_H_

#include <string>

#include "../library/math/matrix_vector.hpp"
#include "../library/math/matrix.hpp"
#include "../library/math/vector.hpp"
#include "../interface/log_output/loggable.hpp"
#include "simple_disturbance.hpp"

using libra::Matrix;
using libra::Vector;

/**
 * @class GravityGradient
 * @brief Class to calculate the gravity gradient torque
 */
class GravityGradient : public SimpleDisturbance {
 public:
  /**
   * @fn GeoPotential
   * @brief Default Constructor
   * @note mu is automatically set as earth's gravitational constant
   */
  GravityGradient();
  /**
   * @fn GeoPotential
   * @brief Constructor
   * @param [in] mu_m3_s2: Gravitational constant [m3/s2]
   */
  GravityGradient(const double mu_m3_s2);

  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
   */
  virtual void Update(const LocalEnvironment& local_env, const Dynamics& dynamics);

  /**
   * @fn CalcTorque
   * @brief Calculate gravity gradient torque
   * @param [in] r_b: Position vector of the earth at body frame [m]
   * @param [in] I_b: Inertia Tensor at body frame [kg*m^2]
   */
  Vector<3> CalcTorque(const Vector<3> r_b, const Matrix<3, 3> I_b);

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
};

#endif  // S2E_DISTURBANCES_GRAVITY_GRADIENT_H_
