/**
 * @file disturbance.hpp
 * @brief Base class for a disturbance
 */

#ifndef S2E_DISTURBANCES_DISTURBANCE_HPP_
#define S2E_DISTURBANCES_DISTURBANCE_HPP_

#include "../library/math/vector.hpp"

/**
 * @class Disturbance
 * @brief Base class for a disturbance
 */
class Disturbance {
 public:
  /**
   * @fn Disturbance
   * @brief Constructor
   * @param [in] is_calculation_enabled: Calculation flag
   */
  Disturbance(const bool is_calculation_enabled = true) : is_calculation_enabled_(is_calculation_enabled) {
    force_b_N_ = libra::Vector<3>(0.0);
    torque_b_Nm_ = libra::Vector<3>(0.0);
    acceleration_b_m_s2_ = libra::Vector<3>(0.0);
    acceleration_b_m_s2_ = libra::Vector<3>(0.0);
  }

  /**
   * @fn GetTorque
   * @brief Return the disturbance torque in the body frame [Nm]
   */
  virtual inline libra::Vector<3> GetTorque_b_Nm() { return torque_b_Nm_; }
  /**
   * @fn GetTorque
   * @brief Return the disturbance force in the body frame [N]
   */
  virtual inline libra::Vector<3> GetForce_b_N() { return force_b_N_; }
  /**
   * @fn GetTorque
   * @brief Return the disturbance acceleration in the body frame [m/s2]
   */
  virtual inline libra::Vector<3> GetAcceleration_b_m_s2() { return acceleration_b_m_s2_; }
  /**
   * @fn GetTorque
   * @brief Return the disturbance acceleration in the inertial frame [m/s2]
   */
  virtual inline libra::Vector<3> GetAcceleration_i_m_s2() { return acceleration_i_m_s2_; }

 protected:
  bool is_calculation_enabled_;           //!< Flag to calculate the disturbance
  libra::Vector<3> force_b_N_;            //!< Disturbance force in the body frame [N]
  libra::Vector<3> torque_b_Nm_;          //!< Disturbance torque in the body frame [Nm]
  libra::Vector<3> acceleration_b_m_s2_;  //!< Disturbance acceleration in the body frame [m/s2]
  libra::Vector<3> acceleration_i_m_s2_;  //!< Disturbance acceleration in the inertial frame [m/s2]
};

#endif  // S2E_DISTURBANCES_DISTURBANCE_HPP_
