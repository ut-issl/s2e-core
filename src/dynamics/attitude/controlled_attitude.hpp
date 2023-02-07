/**
 * @file controlled_attitude.hpp
 * @brief Class to calculate spacecraft attitude with Controlled Attitude mode
 */

#ifndef S2E_DYNAMICS_ATTITUDE_CONTROLLED_ATTITUDE_H_
#define S2E_DYNAMICS_ATTITUDE_CONTROLLED_ATTITUDE_H_

#include <environment/local/local_celestial_information.hpp>
#include <string>

#include "../orbit/orbit.hpp"
#include "attitude.hpp"

/**
 * @enum AttCtrlMode
 * @brief Attitude control mode
 */
enum AttCtrlMode {
  INERTIAL_STABILIZE,           //!< Inertial stabilize
  SUN_POINTING,                 //!< Sun pointing
  EARTH_CENTER_POINTING,        //!< Earth center pointing
  VELOCITY_DIRECTION_POINTING,  //!< Spacecraft velocity direction pointing
  ORBIT_NORMAL_POINTING,        //!< Orbit normal direction pointing
  NO_CTRL,                      // No Control
};

/**
 * @fn ConvertStringToCtrlMode
 * @brief Convert string to AttCtrlMode
 * @param [in] mode: Control mode in string
 * @return Attitude control mode
 */
AttCtrlMode ConvertStringToCtrlMode(const std::string mode);

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
   * @param [in] pointing_t_b: Main target direction on the body fixed frame
   * @param [in] pointing_sub_t_b: Sun target direction on the body fixed frame
   * @param [in] inertia_tensor_kgm2: Inertia tensor of the spacecraft [kg m^2]
   * @param [in] local_celes_info: Local celestial information
   * @param [in] orbit: Orbit
   * @param [in] sim_object_name: Simulation object name for Monte-Carlo simulation
   */
  ControlledAttitude(const AttCtrlMode main_mode, const AttCtrlMode sub_mode, const Quaternion quaternion_i2b, const Vector<3> pointing_t_b,
                     const Vector<3> pointing_sub_t_b, const Matrix<3, 3>& inertia_tensor_kgm2, const LocalCelestialInformation* local_celes_info,
                     const Orbit* orbit, const std::string& sim_object_name = "Attitude");
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
  inline void SetMainMode(const AttCtrlMode main_mode) { main_mode_ = main_mode; }
  /**
   * @fn SetSubMode
   * @brief Set sub control mode
   */
  inline void SetSubMode(const AttCtrlMode sub_mode) { sub_mode_ = sub_mode; }
  /**
   * @fn SetQuaternionI2T
   * @brief Set quaternion for INERTIAL_STABILIZE mode
   */
  inline void SetQuaternionI2T(const Quaternion quaternion_i2t) { quaternion_i2b_ = quaternion_i2t; }
  /**
   * @fn SetPointingTb
   * @brief Set main target direction on the body fixed frame
   */
  inline void SetPointingTb(Vector<3> pointing_t_b) { pointing_t_b_ = pointing_t_b; }
  /**
   * @fn SetPointingSubTb
   * @brief Set sub target direction on the body fixed frame
   */
  inline void SetPointingSubTb(Vector<3> pointing_sub_t_b) { pointing_sub_t_b_ = pointing_sub_t_b; }

  /**
   * @fn Propagate
   * @brief Attitude propagation
   * @param [in] endtime_s: Propagation endtime [sec]
   */
  virtual void Propagate(const double endtime_s);

 private:
  AttCtrlMode main_mode_;                  //!< Main control mode
  AttCtrlMode sub_mode_;                   //!< Sub control mode
  libra::Vector<3> pointing_t_b_;          //!< Main target direction on the body fixed frame
  libra::Vector<3> pointing_sub_t_b_;      //!< Sub target direction on tge body fixed frame
  double previous_calc_time_s_ = -1.0;     //!< Previous time of velocity calculation [sec]
  libra::Quaternion prev_quaternion_i2b_;  //!< Previous quaternion
  libra::Vector<3> prev_omega_b_rad_s_;    //!< Previous angular velocity [rad/s]

  // Inputs
  const LocalCelestialInformation* local_celes_info_;  //!< Local celestial information
  const Orbit* orbit_;                                 //!< Orbit information

  // Local functions
  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize(void);
  /**
   * @fn CalcTargetDirection
   * @brief Calculate target direction from attitude control mode
   * @param [in] mode: Attitude control mode
   */
  Vector<3> CalcTargetDirection(AttCtrlMode mode);
  /**
   * @fn PointingCtrl
   * @brief Calculate attitude quaternion
   * @param [in] main_direction_i: Main target direction in the inertial frame
   * @param [in] main_direction_i: Sub target direction in the inertial frame
   */
  void PointingCtrl(const Vector<3> main_direction_i, const Vector<3> sub_direction_i);
  /**
   * @fn CalcAngularVelocity
   * @brief Calculate angular velocity
   * @param [in] current_time_s: Current time [sec]
   */
  void CalcAngularVelocity(const double current_time_s);
  /**
   * @fn CalcDCM
   * @brief Calculate direction cosine matrix with tow direction vectors
   * @param [in] main_direction: Main target direction
   * @param [in] main_direction: Sub target direction
   */
  Matrix<3, 3> CalcDCM(const Vector<3> main_direction, const Vector<3> sub_direction);
};

#endif  // S2E_DYNAMICS_ATTITUDE_CONTROLLED_ATTITUDE_H_
