/**
 * @file magnetic_disturbance.hpp
 * @brief Class to calculate the magnetic disturbance torque
 */

#ifndef S2E_DISTURBANCES_MAGNETIC_DISTURBANCE_HPP_
#define S2E_DISTURBANCES_MAGNETIC_DISTURBANCE_HPP_

#include "../interface/log_output/loggable.hpp"
#include "../library/math/vector.hpp"
#include "../simulation/spacecraft/structure/residual_magnetic_moment.hpp"
#include "simple_disturbance.hpp"

/**
 * @class MagDisturbance
 * @brief Class to calculate the magnetic disturbance torque
 */
class MagDisturbance : public SimpleDisturbance {
 public:
  /**
   * @fn MagDisturbance
   * @brief Constructor
   * @param [in] is_calculation_enabled: Calculation flag
   */
  MagDisturbance(const RMMParams& rmm_params, const bool is_calculation_enabled = true);

  /**
   * @fn CalcRMM
   * @brief Calculate true RMM of the spacecraft
   */
  void CalcRMM();
  /**
   * @fn CalcTorque
   * @brief Calculate magnetic disturbance torque
   * @param [in] magnetic_field_b_nT: Magnetic field vector at the body frame [nT]
   */
  libra::Vector<3> CalcTorque(const libra::Vector<3>& magnetic_field_b_nT);
  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
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

  libra::Vector<3> rmm_b_Am2_;   //!< True RMM of the spacecraft in the body frame [Am2]
  const RMMParams& rmm_params_;  //!< RMM parameters
};

#endif  // S2E_DISTURBANCES_MAGNETIC_DISTURBANCE_HPP_
