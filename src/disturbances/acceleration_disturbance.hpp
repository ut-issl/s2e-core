/**
 * @file acceleration_disturbance.hpp
 * @brief Abstract class for a disturbance which generate acceleration only (not force)
 */

#ifndef S2E_DISTURBANCES_ACCELERATION_DISTURBANCE_HPP_
#define S2E_DISTURBANCES_ACCELERATION_DISTURBANCE_HPP_

#include "../dynamics/dynamics.hpp"
#include "../environment/local/local_environment.hpp"
#include "disturbance.hpp"

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

#endif  // S2E_DISTURBANCES_ACCELERATION_DISTURBANCE_HPP_
