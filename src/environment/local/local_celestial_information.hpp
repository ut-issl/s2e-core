/**
 * @file local_celestial_information.hpp
 * @brief Class to manage celestial body information in the spacecraft body frame
 */

#ifndef S2E_ENVIRONMENT_LOCAL_LOCAL_CELESTIAL_INFORMATION_HPP_
#define S2E_ENVIRONMENT_LOCAL_LOCAL_CELESTIAL_INFORMATION_HPP_

#include "../global/celestial_information.hpp"

/**
 * @class LocalCelestialInformation
 * @brief Class to manage celestial body information in the spacecraft body frame
 */
class LocalCelestialInformation : public ILoggable {
 public:
  /**
   * @fn LocalCelestialInformation
   * @brief Constructor
   * @param [in] glo_celes_info: Global celestial information
   */
  LocalCelestialInformation(const CelestialInformation* glo_celes_info);
  /**
   * @fn ~LocalCelestialInformation
   * @brief Destructor
   */
  virtual ~LocalCelestialInformation();

  /**
   * @fn UpdateAllObjectsInfo
   * @brief Update the all selected celestial object local information
   * @param [in] sc_pos_from_center_i: Spacecraft position from the center body in the inertial frame [m]
   * @param [in] sc_vel_from_center_i: Spacecraft velocity from the center body in the inertial frame [m/s]
   * @param [in] q_i2b: Spacecraft attitude quaternion from the inertial frame to the body fixed frame
   * @param [in] sc_body_rate: Spacecraft angular velocity with respect to the inertial frame [rad/s]
   */
  void UpdateAllObjectsInfo(const Vector<3> sc_pos_from_center_i, const Vector<3> sc_vel_from_center_i, Quaternion q_i2b,
                            const Vector<3> sc_body_rate);
  /**
   * @fn CalcAllPosVel_b
   * @brief Frame conversion to the body frame for all selected celestial bodies
   * @param [in] q_i2b: Spacecraft attitude quaternion from the inertial frame to the body fixed frame
   * @param [in] sc_body_rate: Spacecraft angular velocity with respect to the inertial frame [rad/s]
   */
  void CalcAllPosVel_b(Quaternion q_i2b, const Vector<3> sc_body_rate);

  /**
   * @fn GetPosFromSC_i
   * @brief Return position of a selected body (Origin: Spacecraft, Frame: Inertial frame)
   * @param [in] body_name Celestial body name
   */
  Vector<3> GetPosFromSC_i(const char* body_name) const;
  /**
   * @fn GetCenterBodyPosFromSC_i
   * @brief Return position of the center body (Origin: Spacecraft, Frame: Inertial frame)
   */
  Vector<3> GetCenterBodyPosFromSC_i(void) const;

  /**
   * @fn GetPosFromSC_b
   * @brief Return position of a selected body (Origin: Spacecraft, Frame: Body fixed frame)
   * @param [in] body_name Celestial body name
   */
  Vector<3> GetPosFromSC_b(const char* body_name) const;
  /**
   * @fn GetCenterBodyPosFromSC_b
   * @brief Return position of the center body (Origin: Spacecraft, Frame: Body fixed frame)
   */
  Vector<3> GetCenterBodyPosFromSC_b(void) const;

  /**
   * @fn GetGlobalInfo
   * @brief Return global celestial information
   */
  inline const CelestialInformation& GetGlobalInfo() const { return *glo_celes_info_; }

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
  const CelestialInformation* glo_celes_info_;  //!< Global celestial information
  // Local Information
  double* celes_objects_pos_from_sc_i_;      //!< Celestial body position from spacecraft in the inertial frame [m]
  double* celes_objects_vel_from_sc_i_;      //!< Celestial body velocity from spacecraft in the inertial frame [m/s]
  double* celes_objects_pos_from_center_b_;  //!< Celestial body position from the center body in the spacecraft body fixed frame [m]
  double* celes_objects_pos_from_sc_b_;      //!< Celestial body position from the spacecraft in the spacecraft body fixed frame [m]
  double* celes_objects_vel_from_center_b_;  //!< Celestial body velocity from the center body in the spacecraft body fixed frame [m/s]
  double* celes_objects_vel_from_sc_b_;      //!< Celestial body velocity from the spacecraft in the spacecraft body fixed frame [m/s]
};

/**
 * @fn Convert_i2b
 * @brief Convert position vector in the inertial frame to the body fixed frame
 * @param [in] src_i: Source vector in the inertial frame
 * @param [out] dst_b: Output vector in the body fixed frame
 * @param [in] q_i2b: Spacecraft attitude quaternion from the inertial frame to the body fixed frame
 */
void Convert_i2b(const double* src_i, double* dst_b, const Quaternion q_i2b);

/**
 * @fn Convert_i2b_velocity
 * @brief Convert velocity vector in the inertial frame to the body fixed frame
 * @param [in] r_i: Position vector in the inertial frame
 * @param [in] v_i: Velocity vector in the inertial frame
 * @param [out] v_b: Output Velocity vector in the body fixed frame
 * @param [in] q_i2b: Spacecraft attitude quaternion from the inertial frame to the body fixed frame
 * @param [in] sc_body_rate: Spacecraft angular velocity with respect to the inertial frame [rad/s]
 */
void Convert_i2b_velocity(const double* r_i, const double* v_i, double* v_b, const Quaternion q_i2b, const Vector<3> bodyrate_b);

#endif  // S2E_ENVIRONMENT_LOCAL_LOCAL_CELESTIAL_INFORMATION_HPP_
