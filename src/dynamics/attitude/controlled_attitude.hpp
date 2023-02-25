/**
 * @file controlled_attitude.hpp
 * @brief Class to calculate spacecraft attitude with Controlled Attitude mode
 */

#ifndef S2E_DYNAMICS_ATTITUDE_CONTROLLED_ATTITUDE_HPP_
#define S2E_DYNAMICS_ATTITUDE_CONTROLLED_ATTITUDE_HPP_

#include <environment/local/local_celestial_information.hpp>
#include <library/math/constants.hpp>
#include <string>

#include "../orbit/orbit.hpp"
#include "attitude.hpp"

/**
 * @enum AttitudeControlMode
 * @brief Attitude control mode
 */
enum AttitudeControlMode {
  INERTIAL_STABILIZE,           //!< Inertial stabilize
  SUN_POINTING,                 //!< Sun pointing
  EARTH_CENTER_POINTING,        //!< Earth center pointing
  VELOCITY_DIRECTION_POINTING,  //!< Spacecraft velocity direction pointing
  ORBIT_NORMAL_POINTING,        //!< Orbit normal direction pointing
  NO_CTRL,                      // No Control
};

/**
 * @fn ConvertStringToCtrlMode
 * @brief Convert string to AttitudeControlMode
 * @param [in] mode: Control mode in string
 * @return Attitude control mode
 */
AttitudeControlMode ConvertStringToCtrlMode(const std::string mode);

/**
 * @class ControlledAttitude
 * @brief Class to calculate spacecraft attitude with Controlled Attitude mode
 */
class ControlledAttitude : public Attitude {
 public:
  /**
   * @fn ControlledAttitude
   * @brief Constructor
   * @param [in] main_mode: Main control mode
   * @param [in] sub_mode: Sub control mode
   * @param [in] quaternion_i2b: Quaternion for INERTIAL_STABILIZE mode
   * @param [in] main_target_direction_b: Main target direction on the body fixed frame
   * @param [in] sub_target_direction_b: Sun target direction on the body fixed frame
   * @param [in] inertia_tensor_kgm2: Inertia tensor of the spacecraft [kg m^2]
   * @param [in] local_celestial_information: Local celestial information
   * @param [in] orbit: Orbit
   * @param [in] simulation_object_name: Simulation object name for Monte-Carlo simulation
   */
  ControlledAttitude(const AttitudeControlMode main_mode, const AttitudeControlMode sub_mode, const Quaternion quaternion_i2b,
                     const Vector<3> main_target_direction_b, const Vector<3> sub_target_direction_b, const Matrix<3, 3>& inertia_tensor_kgm2,
                     const LocalCelestialInformation* local_celestial_information, const Orbit* orbit,
                     const std::string& simulation_object_name = "Attitude");
  /**
   * @fn ~ControlledAttitude
   * @brief Destructor
   */
  ~ControlledAttitude();

  // Setter
  /**
   * @fn SetMainMode
   * @brief Set main control mode
   */
  inline void SetMainMode(const AttitudeControlMode main_mode) { main_mode_ = main_mode; }
  /**
   * @fn SetSubMode
   * @brief Set sub control mode
   */
  inline void SetSubMode(const AttitudeControlMode sub_mode) { sub_mode_ = sub_mode; }
  /**
   * @fn SetQuaternion_i2t
   * @brief Set quaternion for INERTIAL_STABILIZE mode
   */
  inline void SetQuaternion_i2t(const Quaternion quaternion_i2t) { quaternion_i2b_ = quaternion_i2t; }
  /**
   * @fn SetMainTargetDirection_b
   * @brief Set main target direction on the body fixed frame
   */
  inline void SetMainTargetDirection_b(Vector<3> main_target_direction_b) { main_target_direction_b_ = main_target_direction_b; }
  /**
   * @fn SetSubTargetDirection_b
   * @brief Set sub target direction on the body fixed frame
   */
  inline void SetSubTargetDirection_b(Vector<3> sub_target_direction_b) { sub_target_direction_b_ = sub_target_direction_b; }

  /**
   * @fn Propagate
   * @brief Attitude propagation
   * @param [in] end_time_s: Propagation endtime [sec]
   */
  virtual void Propagate(const double end_time_s);

 private:
  AttitudeControlMode main_mode_;              //!< Main control mode
  AttitudeControlMode sub_mode_;               //!< Sub control mode
  libra::Vector<3> main_target_direction_b_;   //!< Main target direction on the body fixed frame
  libra::Vector<3> sub_target_direction_b_;    //!< Sub target direction on tge body fixed frame
  double previous_calc_time_s_ = -1.0;         //!< Previous time of velocity calculation [sec]
  libra::Quaternion previous_quaternion_i2b_;  //!< Previous quaternion
  libra::Vector<3> previous_omega_b_rad_s_;    //!< Previous angular velocity [rad/s]

  const double kMinDirectionAngle_rad = 30.0 * libra::deg_to_rad;  //!< Minimum angle b/w main and sub direction
                                                                               // TODO Change with ini file

  // Inputs
  const LocalCelestialInformation* local_celestial_information_;  //!< Local celestial information
  const Orbit* orbit_;                                            //!< Orbit information

  // Local functions
  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize(void);
  /**
   * @fn CalcTargetDirection_i
   * @brief Calculate target direction from attitude control mode
   * @param [in] mode: Attitude control mode
   * @return Target direction at the inertia frame0
   */
  Vector<3> CalcTargetDirection_i(AttitudeControlMode mode);
  /**
   * @fn PointingControl
   * @brief Calculate attitude quaternion
   * @param [in] main_direction_i: Main target direction in the inertial frame
   * @param [in] sub_direction_i: Sub target direction in the inertial frame
   */
  void PointingControl(const Vector<3> main_direction_i, const Vector<3> sub_direction_i);
  /**
   * @fn CalcAngularVelocity
   * @brief Calculate angular velocity
   * @param [in] current_time_s: Current time [sec]
   */
  void CalcAngularVelocity(const double current_time_s);
  /**
   * @fn CalcDcm
   * @brief Calculate direction cosine matrix with tow direction vectors
   * @param [in] main_direction: Main target direction
   * @param [in] sub_direction: Sub target direction
   */
  Matrix<3, 3> CalcDcm(const Vector<3> main_direction, const Vector<3> sub_direction);
};

#endif  // S2E_DYNAMICS_ATTITUDE_CONTROLLED_ATTITUDE_HPP_
