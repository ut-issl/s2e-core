/**
 * @file disturbance.hpp
 * @brief Base class for a disturbance
 */

#ifndef S2E_DISTURBANCES_DISTURBANCE_HPP_
#define S2E_DISTURBANCES_DISTURBANCE_HPP_

#include "../environment/local/local_environment.hpp"
#include "../math_physics/math/vector.hpp"

/**
 * @class Disturbance
 * @brief Base class for a disturbance
 */
class Disturbance : public ILoggable {
 public:
  /**
   * @fn Disturbance
   * @brief Constructor
   * @param [in] is_calculation_enabled: Calculation flag
   * @param [in] is_attitude_dependent: Attitude dependent flag
   */
  Disturbance(const bool is_calculation_enabled = true, const bool is_attitude_dependent = true)
      : is_calculation_enabled_(is_calculation_enabled), is_attitude_dependent_(is_attitude_dependent) {
    force_b_N_ = s2e::math::Vector<3>(0.0);
    torque_b_Nm_ = s2e::math::Vector<3>(0.0);
    acceleration_i_m_s2_ = s2e::math::Vector<3>(0.0);
    acceleration_b_m_s2_ = s2e::math::Vector<3>(0.0);
  }

  /**
   * @fn ~Disturbance
   * @brief Destructor
   */
  virtual ~Disturbance() {}

  /**
   * @fn UpdateIfEnabled
   * @brief Update calculated disturbance when the calculation flag is true
   */
  virtual inline void UpdateIfEnabled(const LocalEnvironment& local_environment, const Dynamics& dynamics) {
    if (is_calculation_enabled_) {
      Update(local_environment, dynamics);
    } else {
      force_b_N_ *= 0.0;
      torque_b_Nm_ *= 0.0;
      acceleration_b_m_s2_ *= 0.0;
      acceleration_i_m_s2_ *= 0.0;
    }
  }

  /**
   * @fn Update
   * @brief Pure virtual function to define the disturbance calculation
   */
  virtual void Update(const LocalEnvironment& local_environment, const Dynamics& dynamics) = 0;

  /**
   * @fn GetTorque_b_Nm
   * @brief Return the disturbance torque in the body frame [Nm]
   */
  virtual inline s2e::math::Vector<3> GetTorque_b_Nm() { return torque_b_Nm_; }
  /**
   * @fn GetForce_b_N
   * @brief Return the disturbance force in the body frame [N]
   */
  virtual inline s2e::math::Vector<3> GetForce_b_N() { return force_b_N_; }
  /**
   * @fn GetAcceleration_b_m_s2
   * @brief Return the disturbance acceleration in the body frame [m/s2]
   */
  virtual inline s2e::math::Vector<3> GetAcceleration_b_m_s2() { return acceleration_b_m_s2_; }
  /**
   * @fn GetAcceleration_i_m_s2
   * @brief Return the disturbance acceleration in the inertial frame [m/s2]
   */
  virtual inline s2e::math::Vector<3> GetAcceleration_i_m_s2() { return acceleration_i_m_s2_; }
  /**
   * @fn IsAttitudeDependent
   * @brief Return the attitude dependent flag
   */
  virtual inline bool IsAttitudeDependent() { return is_attitude_dependent_; }

 protected:
  bool is_calculation_enabled_;          //!< Flag to calculate the disturbance
  bool is_attitude_dependent_;           //!< Flag to show the disturbance depends on attitude information
  s2e::math::Vector<3> force_b_N_;            //!< Disturbance force in the body frame [N]
  s2e::math::Vector<3> torque_b_Nm_;          //!< Disturbance torque in the body frame [Nm]
  s2e::math::Vector<3> acceleration_b_m_s2_;  //!< Disturbance acceleration in the body frame [m/s2]
  s2e::math::Vector<3> acceleration_i_m_s2_;  //!< Disturbance acceleration in the inertial frame [m/s2]
};

#endif  // S2E_DISTURBANCES_DISTURBANCE_HPP_
