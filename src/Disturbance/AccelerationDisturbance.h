/**
 * @file AccelerationDisturbance.h
 * @brief Abstract class for a disturbance which generate acceleration only (not force)
 */

#pragma once
#include "../Dynamics/Dynamics.h"
#include "../Environment/Local/LocalEnvironment.h"
#include "Disturbance.h"

/**
 * @class AccelerationDisturbance
 * @brief Abstract class for a disturbance which generate acceleration only (not force)
 */
class AccelerationDisturbance : public Disturbance, public ILoggable {
 public:
  /**
   * @fn ~AccelerationDisturbance
   * @brief Destructor
   */
  virtual ~AccelerationDisturbance() {}

  /**
   * @fn UpdateIfEnabled
   * @brief Update calculated disturbance when the calculation flag is true
   */
  virtual inline void UpdateIfEnabled(const LocalEnvironment& local_env, const Dynamics& dynamics) {
    if (IsCalcEnabled) {
      Update(local_env, dynamics);
    } else {
      acceleration_b_ *= 0;
      acceleration_i_ *= 0;
    }
  }

  /**
   * @fn Update
   * @brief Pure virtual function to define the disturbance calculation
   */
  virtual void Update(const LocalEnvironment& local_env, const Dynamics& dynamics) = 0;
};
