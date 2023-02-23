/**
 * @file celestial_information.hpp
 * @brief Class to manage the information related with the celestial bodies
 * @details This class uses SPICE to get the information of celestial bodies
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_CELESTIAL_INFORMATION_HPP_
#define S2E_ENVIRONMENT_GLOBAL_CELESTIAL_INFORMATION_HPP_

#include "celestial_rotation.hpp"
#include "library/logger/loggable.hpp"
#include "library/math/vector.hpp"

/**
 * @class CelestialInformation
 * @brief Class to manage the information related with the celestial bodies
 * @details This class uses SPICE to get the information of celestial bodies
 */
class CelestialInformation : public ILoggable {
 public:
  /**
   * @fn CelestialInformation
   * @brief Constructor
   * @param [in] inertial_frame_name:  Definition of inertial frame
   * @param [in] aberration_correction_setting: Stellar aberration correction
   * @param [in] center_body_name: Center body name of inertial frame
   * @param [in] rotation_mode: Designation of rotation model
   * @param [in] number_of_selected_body: Number of selected body
   * @param [in] selected_body_id: SPICE IDs of selected bodies
   */
  CelestialInformation(std::string inertial_frame_name, std::string aberration_correction_setting, std::string center_body_name,
                       RotationMode rotation_mode, unsigned int number_of_selected_body, int* selected_body_id);
  /**
   * @fn CelestialInformation
   * @brief Copy constructor
   */
  CelestialInformation(const CelestialInformation& obj);
  /**
   * @fn ~CelestialInformation
   * @brief Destructor
   */
  virtual ~CelestialInformation();

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

  /**
   * @fn UpdateAllObjectsInfo
   * @brief Update the information of all selected celestial objects
   */
  void UpdateAllObjectsInfo(const double current_jd);

  // Getters
  // Orbit information
  /**
   * @fn GetPosFromCenter_i
   * @brief Return position from the center body in the inertial frame [m]
   * @param [in] id: ID of CelestialInformation list
   */
  libra::Vector<3> GetPosFromCenter_i(const int id) const;
  /**
   * @fn GetPosFromCenter_i
   * @brief Return position from the center body in the inertial frame [m]
   * @param [in] body_name: Name of the body defined in the SPICE
   */
  libra::Vector<3> GetPosFromCenter_i(const char* body_name) const;
  /**
   * @fn GetVelFromCenter_i
   * @brief Return velocity from the center body in the inertial frame [m/s]
   * @param [in] id: ID of CelestialInformation list
   */
  libra::Vector<3> GetVelFromCenter_i(const int id) const;
  /**
   * @fn GetVelFromCenter_i
   * @brief Return velocity from the center body in the inertial frame [m/s]
   * @param [in] body_name: Name of the body defined in the SPICE
   */
  libra::Vector<3> GetVelFromCenter_i(const char* body_name) const;

  // Gravity constants
  /**
   * @fn GetGravityConstant
   * @brief Return gravity constant of the celestial body [m^3/s^2]
   * @param [in] body_name: Name of the body defined in the SPICE
   */
  double GetGravityConstant(const char* body_name) const;
  /**
   * @fn GetCenterBodyGravityConstant_m3_s2
   * @brief Return gravity constant of the center body [m^3/s^2]
   */
  double GetCenterBodyGravityConstant_m3_s2(void) const;

  // Shape information
  /**
   * @fn GetRadii
   * @brief Return 3 axis planetographic radii of a celestial body [m]
   * @param [in] id: ID of CelestialInformation list
   */
  libra::Vector<3> GetRadii(const int id) const;
  /**
   * @fn GetRadiiFromName
   * @brief Return 3 axis planetographic radii of a celestial body [m]
   * @param [in] body_name: Name of the body defined in the SPICE
   */
  libra::Vector<3> GetRadiiFromName(const char* body_name) const;
  /**
   * @fn GetMeanRadiusFromName
   * @brief Return mean radius of a celestial body [m]
   * @param [in] id: ID of CelestialInformation list
   */
  double GetMeanRadiusFromName(const char* body_name) const;

  // Parameters
  /**
   * @fn GetNumBody
   * @brief Return number of selected body
   */
  inline unsigned int GetNumBody(void) const { return number_of_selected_body_id_; }
  /**
   * @fn GetSelectedBody
   * @brief Return SPICE IDs of selected bodies
   */
  inline int* GetSelectedBody(void) const { return selected_body_id_; }
  /**
   * @fn GetCenterBodyName
   * @brief Return name of the center body
   */
  inline std::string GetCenterBodyName(void) const { return center_body_name_; }

  // Members
  /**
   * @fn GetEarthRotation
   * @brief Return EarthRotation information
   */
  inline CelestialRotation GetEarthRotation(void) const { return *earth_rotation_; };

  // Calculation
  /**
   * @fn CalcBodyIdFromName
   * @brief Acquisition of ID of CelestialInformation list from body name
   * @param [in] body_name: Celestial body name
   * @return ID of CelestialInformation list
   */
  int CalcBodyIdFromName(const char* body_name) const;
  /**
   * @fn DebugOutput
   * @brief Debug output
   */
  void DebugOutput(void);

 private:
  // Setting parameters
  unsigned int number_of_selected_body_id_;    //!< Number of selected body
  int* selected_body_id_;                      //!< SPICE IDs of selected bodies
  std::string inertial_frame_name_;            //!< Definition of inertial frame
  std::string center_body_name_;               //!< Center object name of inertial frame
  std::string aberration_correction_setting_;  //!< Stellar aberration correction
                                               //!< Ref：http://fermi.gsfc.nasa.gov/ssc/library/fug/051108/Aberration_Julie.ppt

  // Calculated values
  double* celestial_body_position_from_center_i_m_;    //!< Position vector list at inertial frame [m]
  double* celestial_body_velocity_from_center_i_m_s_;  //!< Velocity vector list at inertial frame [m/s]
  double* celestial_body_gravity_constant_m3_s2_;      //!< Gravity constant list [m^3/s^2]
  double* celestial_body_mean_radius_m_;               //!< Mean radius list [m] r = (rx * ry * rz)^(1/3)
  double* celestial_body_planetographic_radii_m_;      //!< 3 axis planetographic radii [m]
                                                       // X-axis pass through the 0 degree latitude 0 degree longitude direction
                                                       // Z-axis pass through the 90 degree latitude direction
                                                       // Y-axis equal to the cross product of the unit Z-axis and X-axis vectors

  // Rotational Motion of each planets
  CelestialRotation* earth_rotation_;  //!< Instance of Earth rotation
  RotationMode rotation_mode_;         //!< Designation of rotation model

  /**
   * @fn GetPlanetOrbit
   * @brief Get position/velocity of planet.
   * @note This is an override function of SPICE's spkezr_c (https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/cspice/spkezr_c.html)
   * @param [in] planet_name: Nama of planet defined by SPICE
   * @param [in] et: Ephemeris time
   * @param [out] orbit: Cartesian state vector representing the position and velocity of the target body relative to the specified observer.
   */
  void GetPlanetOrbit(const char* planet_name, double et, double orbit[6]);
};

#endif  // S2E_ENVIRONMENT_GLOBAL_CELESTIAL_INFORMATION_HPP_
