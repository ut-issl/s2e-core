/**
 * @file MagDisturbance.h
 * @brief Class to calculate the magnetic disturbance torque
 */

#ifndef __MagDisturbance_H__
#define __MagDisturbance_H__

#include <string>

#include "../Library/math/Vector.hpp"
using libra::Vector;

#include "../Interface/LogOutput/ILoggable.h"
#include "../Simulation/Spacecraft/Structure/RMMParams.h"
#include "SimpleDisturbance.h"

/**
 * @class MagDisturbance
 * @brief Class to calculate the magnetic disturbance torque
 */
class MagDisturbance : public SimpleDisturbance {
  Vector<3> rmm_b_;        //!< True RMM of the spacecraft in the body frame [Am2]
  double mag_unit_;        //!< Constant value to change the unit [nT] -> [T]
  Vector<3> rmm_const_b_;  //!< Constant component of the RMM in the body frame [Am2]
  double rmm_rwdev_;       //!< Standard deviation of random walk component of the RMM [Am2]
  double rmm_rwlimit_;     //!< Limit of random walk component of the RMM [Am2]
  double rmm_wnvar_;       //!< Standard deviation of white noise of the RMM [Am2]

 public:
  /**
   * @fn MagDisturbance
   * @brief Constructor
   */
  MagDisturbance(const Vector<3>& rmm_const_b, const double rmm_rwdev, const double rmm_rwlimit, const double rmm_wnvar);

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
};

#endif  //__MagDisturbance_H__
