/**
 * @file third_body_gravity.hpp
 * @brief Class to calculate third body gravity disturbance
 */

#ifndef S2E_DISTURBANCES_THIRD_BODY_GRAVITY_H_
#define S2E_DISTURBANCES_THIRD_BODY_GRAVITY_H_

#include <cassert>
#include <set>
#include <string>

#include "../library/math/Vector.hpp"
#include "../interface/log_output/loggable.hpp"
#include "acceleration_disturbance.hpp"

/**
 * @class ThirdBodyGravity
 * @brief Class to calculate third body gravity disturbance
 */
class ThirdBodyGravity : public AccelerationDisturbance {
 public:
  /**
   * @fn ThirdBodyGravity
   * @brief Constructor
   */
  ThirdBodyGravity(std::set<std::string> third_body_list);
  /**
   * @fn ~ThirdBodyGravity
   * @brief Destructor
   */
  ~ThirdBodyGravity();

  /**
   * @fn Update
   * @brief Update third body disturbance
   */
  virtual void Update(const LocalEnvironment& local_env, const Dynamics& dynamics);

 private:
  // Override classes for ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override function of GetLogHeader
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override function of GetLogValue
   */
  virtual std::string GetLogValue() const;

  /**
   * @fn CalcAcceleration
   * @brief Calculate and return the third body disturbance acceleration
   * @param [in] s: Position vector of the third celestial body from the origin in the inertial frame in unit [m]
   * @param [in] sr: Position vector of the third celestial body from the spacecraft in the inertial frame in unit [m]
   * @param [in] GM: The gravitational constants of the third celestial body [m3/s2]
   * @return Third body disturbance acceleration in the inertial frame in unit [m/s2]
   */
  libra::Vector<3> CalcAcceleration(libra::Vector<3> s, libra::Vector<3> sr, double GM);

  std::set<std::string> third_body_list_;  //!< List of celestial bodies to calculate the third body disturbances
  libra::Vector<3> thirdbody_acc_i_{0};    //!< Calculated third body disturbance acceleration in the inertial frame [m/s2]
};

#endif  // S2E_DISTURBANCES_THIRD_BODY_GRAVITY_H_
