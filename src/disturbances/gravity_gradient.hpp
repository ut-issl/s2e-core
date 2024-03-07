/**
 * @file gravity_gradient.hpp
 * @brief Class to calculate the gravity gradient torque
 */

#ifndef S2E_DISTURBANCES_GRAVITY_GRADIENT_HPP_
#define S2E_DISTURBANCES_GRAVITY_GRADIENT_HPP_

#include <string>

#include "../math_physics/math/matrix.hpp"
#include "../math_physics/math/matrix_vector.hpp"
#include "../math_physics/math/vector.hpp"
#include "../logger/loggable.hpp"
#include "disturbance.hpp"

/**
 * @class GravityGradient
 * @brief Class to calculate the gravity gradient torque
 */
class GravityGradient : public Disturbance {
 public:
  /**
   * @fn GravityGradient
   * @brief Default Constructor
   * @param [in] is_calculation_enabled: Calculation flag
   * @note mu is automatically set as earth's gravitational constant
   */
  GravityGradient(const bool is_calculation_enabled = true);
  /**
   * @fn GravityGradient
   * @brief Constructor
   * @param [in] gravity_constant_m3_s2: Gravitational constant [m3/s2]
   * @param [in] is_calculation_enabled: Calculation flag
   */
  GravityGradient(const double gravity_constant_m3_s2, const bool is_calculation_enabled = true);

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
  double gravity_constant_m3_s2_;  //!< Gravitational constant [m3/s2]

  /**
   * @fn CalcTorque
   * @brief Calculate gravity gradient torque
   * @param [in] earth_position_from_sc_b_m: Position vector of the earth from spacecraft at body frame [m]
   * @param [in] inertia_tensor_b_kgm2: Inertia Tensor at body frame [kg*m^2]
   * @return Calculated torque at body frame [Nm]
   */
  libra::Vector<3> CalcTorque_b_Nm(const libra::Vector<3> earth_position_from_sc_b_m, const libra::Matrix<3, 3> inertia_tensor_b_kgm2);
};

/**
 * @fn InitGravityGradient
 * @brief Initialize GravityGradient class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 */
GravityGradient InitGravityGradient(const std::string initialize_file_path);

/**
 * @fn InitGravityGradient
 * @brief Initialize GravityGradient class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 * @param [in] gravity_constant_m3_s2: Gravitational constant [m3/s2]
 */
GravityGradient InitGravityGradient(const std::string initialize_file_path, const double gravity_constant_m3_s2);

#endif  // S2E_DISTURBANCES_GRAVITY_GRADIENT_HPP_
