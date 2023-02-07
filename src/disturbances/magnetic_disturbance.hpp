/**
 * @file magnetic_disturbance.hpp
 * @brief Class to calculate the magnetic disturbance torque
 */

#ifndef S2E_DISTURBANCES_MAGNETIC_DISTURBANCE_H_
#define S2E_DISTURBANCES_MAGNETIC_DISTURBANCE_H_

#include <string>

#include "../Library/math/Vector.hpp"
using libra::Vector;

#include "../Interface/LogOutput/ILoggable.h"
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
   */
  MagDisturbance(const RMMParams& rmm_params);

  /**
   * @fn CalcRMM
   * @brief Calculate true RMM of the spacecraft
   */
  void CalcRMM();
  /**
   * @fn CalcTorque
   * @brief Calculate magnetic disturbance torque
   * @param [in] mag_b: Magnetic field vector at the body frame [nT]
   */
  Vector<3> CalcTorque(const Vector<3>& mag_b);
  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
   */
  virtual void Update(const LocalEnvironment& local_env, const Dynamics& dynamics);
  /**
   * @fn PrintTorque
   * @brief Debug TODO: remove?
   */
  void PrintTorque();

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
  double mag_unit_;  //!< Constant value to change the unit [nT] -> [T]

  Vector<3> rmm_b_;              //!< True RMM of the spacecraft in the body frame [Am2]
  const RMMParams& rmm_params_;  //!< RMM parameters
};

#endif  // S2E_DISTURBANCES_MAGNETIC_DISTURBANCE_H_
