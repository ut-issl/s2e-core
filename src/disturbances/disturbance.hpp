/**
 * @file disturbance.hpp
 * @brief Base class for a disturbance
 */

#ifndef S2E_DISTURBANCES_DISTURBANCE_H_
#define S2E_DISTURBANCES_DISTURBANCE_H_

#include "../library/math/vector.hpp"
using libra::Vector;

/**
 * @class Disturbance
 * @brief Base class for a disturbance
 */
class Disturbance {
 public:
  /**
   * @fn Disturbance
   * @brief Constructor
   */
  Disturbance() {
    force_b_ = Vector<3>(0);
    torque_b_ = Vector<3>(0);
    acceleration_b_ = Vector<3>(0);
    acceleration_b_ = Vector<3>(0);
  }

  /**
   * @fn GetTorque
   * @brief Return the disturbance torque in the body frame [Nm]
   */
  virtual inline Vector<3> GetTorque() { return torque_b_; }
  /**
   * @fn GetTorque
   * @brief Return the disturbance force in the body frame [N]
   */
  virtual inline Vector<3> GetForce() { return force_b_; }
  /**
   * @fn GetTorque
   * @brief Return the disturbance acceleration in the body frame [m/s2]
   */
  virtual inline Vector<3> GetAccelerationB() { return acceleration_b_; }
  /**
   * @fn GetTorque
   * @brief Return the disturbance acceleration in the inertial frame [m/s2]
   */
  virtual inline Vector<3> GetAccelerationI() { return acceleration_i_; }

  bool IsCalcEnabled = true;  //!< Flag to calculate the disturbance

 protected:
  Vector<3> force_b_;         //!< Disturbance force in the body frame [N]
  Vector<3> torque_b_;        //!< Disturbance torque in the body frame [Nm]
  Vector<3> acceleration_b_;  //!< Disturbance acceleration in the body frame [m/s2]
  Vector<3> acceleration_i_;  //!< Disturbance acceleration in the inertial frame [m/s2]
};

#endif  // S2E_DISTURBANCES_DISTURBANCE_H_
