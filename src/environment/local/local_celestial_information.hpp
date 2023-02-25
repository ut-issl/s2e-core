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
   * @param [in] global_celestial_information: Global celestial information
   */
  LocalCelestialInformation(const CelestialInformation* global_celestial_information);
  /**
   * @fn ~LocalCelestialInformation
   * @brief Destructor
   */
  virtual ~LocalCelestialInformation();

  /**
   * @fn UpdateAllObjectsInfo
   * @brief Update the all selected celestial object local information
   * @param [in] spacecraft_position_from_center_i_m: Spacecraft position from the center body in the inertial frame [m]
   * @param [in] spacecraft_velocity_from_center_i_m_s: Spacecraft velocity from the center body in the inertial frame [m/s]
   * @param [in] quaternion_i2b: Spacecraft attitude quaternion from the inertial frame to the body fixed frame
   * @param [in] spacecraft_angular_velocity_rad_s: Spacecraft angular velocity with respect to the inertial frame [rad/s]
   */
  void UpdateAllObjectsInformation(const libra::Vector<3> spacecraft_position_from_center_i_m,
                                   const libra::Vector<3> spacecraft_velocity_from_center_i_m_s, const libra::Quaternion quaternion_i2b,
                                   const libra::Vector<3> spacecraft_angular_velocity_rad_s);

  /**
   * @fn GetPositionFromSpacecraft_i_m
   * @brief Return position of a selected body (Origin: Spacecraft, Frame: Inertial frame)
   * @param [in] body_name Celestial body name
   */
  libra::Vector<3> GetPositionFromSpacecraft_i_m(const char* body_name) const;
  /**
   * @fn GetCenterBodyPositionFromSpacecraft_i_m
   * @brief Return position of the center body (Origin: Spacecraft, Frame: Inertial frame)
   */
  libra::Vector<3> GetCenterBodyPositionFromSpacecraft_i_m(void) const;

  /**
   * @fn GetPositionFromSpacecraft_b_m
   * @brief Return position of a selected body (Origin: Spacecraft, Frame: Body fixed frame)
   * @param [in] body_name Celestial body name
   */
  libra::Vector<3> GetPositionFromSpacecraft_b_m(const char* body_name) const;
  /**
   * @fn GetCenterBodyPositionFromSpacecraft_b_m
   * @brief Return position of the center body (Origin: Spacecraft, Frame: Body fixed frame)
   */
  libra::Vector<3> GetCenterBodyPositionFromSpacecraft_b_m(void) const;

  /**
   * @fn GetGlobalInfo
   * @brief Return global celestial information
   */
  inline const CelestialInformation& GetGlobalInformation() const { return *global_celestial_information_; }

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
  const CelestialInformation* global_celestial_information_;  //!< Global celestial information
  // Local Information
  double* celestial_body_position_from_spacecraft_i_m_;    //!< Celestial body position from spacecraft in the inertial frame [m]
  double* celestial_body_velocity_from_spacecraft_i_m_s_;  //!< Celestial body velocity from spacecraft in the inertial frame [m/s]
  double* celestial_body_position_from_center_b_m_;        //!< Celestial body position from the center body in the spacecraft body fixed frame [m]
  double* celestial_body_position_from_spacecraft_b_m_;    //!< Celestial body position from the spacecraft in the spacecraft body fixed frame [m]
  double* celestial_body_velocity_from_center_b_m_s_;      //!< Celestial body velocity from the center body in the spacecraft body fixed frame [m/s]
  double* celestial_body_velocity_from_spacecraft_b_m_s_;  //!< Celestial body velocity from the spacecraft in the spacecraft body fixed frame [m/s]

  /**
   * @fn CalcAllPosVel_b
   * @brief Frame conversion to the body frame for all selected celestial bodies
   * @param [in] quaternion_i2b: Spacecraft attitude quaternion from the inertial frame to the body fixed frame
   * @param [in] spacecraft_angular_velocity_rad_s: Spacecraft angular velocity with respect to the inertial frame [rad/s]
   */
  void CalcAllPosVel_b(const libra::Quaternion quaternion_i2b, const libra::Vector<3> spacecraft_angular_velocity_rad_s);

  /**
   * @fn ConvertInertialToBody
   * @brief Convert position vector in the inertial frame to the body fixed frame
   * @param [in] input_i: Source vector in the inertial frame
   * @param [out] output_b: Output vector in the body fixed frame
   * @param [in] quaternion_i2b: Spacecraft attitude quaternion from the inertial frame to the body fixed frame
   */
  void ConvertInertialToBody(const double* input_i, double* output_b, const libra::Quaternion quaternion_i2b);

  /**
   * @fn ConvertVelocityInertialToBody
   * @brief Convert velocity vector in the inertial frame to the body fixed frame
   * @param [in] position_i: Position vector in the inertial frame
   * @param [in] velocity_i: Velocity vector in the inertial frame
   * @param [out] velocity_b: Output Velocity vector in the body fixed frame
   * @param [in] quaternion_i2b: Spacecraft attitude quaternion from the inertial frame to the body fixed frame
   * @param [in] angular_velocity_b: Spacecraft angular velocity with respect to the inertial frame [rad/s]
   */
  void ConvertVelocityInertialToBody(const double* position_i, const double* velocity_i, double* velocity_b, const libra::Quaternion quaternion_i2b,
                                     const libra::Vector<3> angular_velocity_b);
};

#endif  // S2E_ENVIRONMENT_LOCAL_LOCAL_CELESTIAL_INFORMATION_HPP_
