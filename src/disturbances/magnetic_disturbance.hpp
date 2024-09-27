/**
 * @file magnetic_disturbance.hpp
 * @brief Class to calculate the magnetic disturbance torque
 */

#ifndef S2E_DISTURBANCES_MAGNETIC_DISTURBANCE_HPP_
#define S2E_DISTURBANCES_MAGNETIC_DISTURBANCE_HPP_

#include <string>

#include "../logger/loggable.hpp"
#include "../math_physics/math/vector.hpp"
#include "../simulation/spacecraft/structure/residual_magnetic_moment.hpp"
#include "disturbance.hpp"

namespace s2e::disturbances {

/**
 * @class MagneticDisturbance
 * @brief Class to calculate the magnetic disturbance torque
 */
class MagneticDisturbance : public Disturbance {
 public:
  /**
   * @fn MagneticDisturbance
   * @brief Constructor
   * @param [in] rmm_parameters: RMM parameters of the spacecraft
   * @param [in] is_calculation_enabled: Calculation flag
   */
  MagneticDisturbance(const ResidualMagneticMoment& rmm_parameters, const bool is_calculation_enabled = true);

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
  const double kMagUnit_ = 1.0e-9;  //!< Constant value to change the unit [nT] -> [T]

  s2e::math::Vector<3> rmm_b_Am2_;                               //!< True RMM of the spacecraft in the body frame [Am2]
  const ResidualMagneticMoment& residual_magnetic_moment_;  //!< RMM parameters

  /**
   * @fn CalcRMM
   * @brief Calculate true RMM of the spacecraft
   */
  void CalcRMM();
  /**
   * @fn CalcTorque_b_Nm
   * @brief Calculate magnetic disturbance torque
   * @param [in] magnetic_field_b_nT: Magnetic field vector at the body frame [nT]
   * @return Calculated disturbance torque in body frame [Nm]
   */
  s2e::math::Vector<3> CalcTorque_b_Nm(const s2e::math::Vector<3>& magnetic_field_b_nT);
};

/**
 * @fn InitMagneticDisturbance
 * @brief Initialize MagneticDisturbance class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 * @param [in] rmm_params: RMM parameters
 */
MagneticDisturbance InitMagneticDisturbance(const std::string initialize_file_path, const ResidualMagneticMoment& rmm_params);

} // namespace s2e::disturbances

#endif  // S2E_DISTURBANCES_MAGNETIC_DISTURBANCE_HPP_
