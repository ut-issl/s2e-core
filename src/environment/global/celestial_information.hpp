/**
 * @file celestial_information.hpp
 * @brief Class to manage the information related with the celestial bodies
 * @details This class uses SPICE to get the information of celestial bodies
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_CELESTIAL_INFORMATION_HPP_
#define S2E_ENVIRONMENT_GLOBAL_CELESTIAL_INFORMATION_HPP_

#include "earth_rotation.hpp"
#include "library/logger/loggable.hpp"
#include "library/math/vector.hpp"
#include "simulation_time.hpp"

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
   * @param [in] selected_body_ids: SPICE IDs of selected bodies
   */
  CelestialInformation(const std::string inertial_frame_name, const std::string aberration_correction_setting, const std::string center_body_name,
                       const RotationMode rotation_mode, const unsigned int number_of_selected_body, int* selected_body_ids);
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
   * @fn UpdateAllObjectsInformation
   * @brief Update the information of all selected celestial objects
   * @param [in] simulation_time: Simulation Time information
   */
  void UpdateAllObjectsInformation(const SimulationTime& simulation_time);

  // Getters
  // Orbit information
  /**
   * @fn GetPositionFromCenter_i_m
   * @brief Return position from the center body in the inertial frame [m]
   * @param [in] id: ID of CelestialInformation list
   */
  inline libra::Vector<3> GetPositionFromCenter_i_m(const unsigned int id) const {
    libra::Vector<3> pos(0.0);
    if (id > number_of_selected_bodies_) return pos;
    for (int i = 0; i < 3; i++) pos[i] = celestial_body_position_from_center_i_m_[id * 3 + i];
    return pos;
  }
  /**
   * @fn GetPositionFromCenter_i_m
   * @brief Return position from the center body in the inertial frame [m]
   * @param [in] body_name: Name of the body defined in the SPICE
   */
  inline libra::Vector<3> GetPositionFromCenter_i_m(const char* body_name) const {
    int id = CalcBodyIdFromName(body_name);
    return GetPositionFromCenter_i_m(id);
  }

  /**
   * @fn GetVelocityFromCenter_i_m_s
   * @brief Return velocity from the center body in the inertial frame [m/s]
   * @param [in] id: ID of CelestialInformation list
   */
  inline libra::Vector<3> GetVelocityFromCenter_i_m_s(const unsigned int id) const {
    libra::Vector<3> vel(0.0);
    if (id > number_of_selected_bodies_) return vel;
    for (int i = 0; i < 3; i++) vel[i] = celestial_body_velocity_from_center_i_m_s_[id * 3 + i];
    return vel;
  }
  /**
   * @fn GetVelocityFromCenter_i_m_s
   * @brief Return velocity from the center body in the inertial frame [m/s]
   * @param [in] body_name: Name of the body defined in the SPICE
   */
  inline libra::Vector<3> GetVelocityFromCenter_i_m_s(const char* body_name) const {
    int id = CalcBodyIdFromName(body_name);
    return GetVelocityFromCenter_i_m_s(id);
  }

  // Gravity constants
  /**
   * @fn GetGravityConstant_m3_s2
   * @brief Return gravity constant of the celestial body [m^3/s^2]
   * @param [in] body_name: Name of the body defined in the SPICE
   */
  inline double GetGravityConstant_m3_s2(const char* body_name) const {
    int index = CalcBodyIdFromName(body_name);
    return celestial_body_gravity_constant_m3_s2_[index];
  }
  /**
   * @fn GetCenterBodyGravityConstant_m3_s2
   * @brief Return gravity constant of the center body [m^3/s^2]
   */
  inline double GetCenterBodyGravityConstant_m3_s2(void) const { return GetGravityConstant_m3_s2(center_body_name_.c_str()); }

  // Shape information
  /**
   * @fn GetRadii_m
   * @brief Return 3 axis planetographic radii of a celestial body [m]
   * @param [in] id: ID of CelestialInformation list
   */
  inline libra::Vector<3> GetRadii_m(const unsigned int id) const {
    libra::Vector<3> radii(0.0);
    if (id > number_of_selected_bodies_) return radii;
    for (int i = 0; i < 3; i++) radii[i] = celestial_body_planetographic_radii_m_[id * 3 + i];
    return radii;
  }
  /**
   * @fn GetRadiiFromName_m
   * @brief Return 3 axis planetographic radii of a celestial body [m]
   * @param [in] body_name: Name of the body defined in the SPICE
   */
  inline libra::Vector<3> GetRadiiFromName_m(const char* body_name) const {
    int id = CalcBodyIdFromName(body_name);
    return GetRadii_m(id);
  }
  /**
   * @fn GetMeanRadiusFromName_m
   * @brief Return mean radius of a celestial body [m]
   * @param [in] id: ID of CelestialInformation list
   */
  inline double GetMeanRadiusFromName_m(const char* body_name) const {
    int index = CalcBodyIdFromName(body_name);
    return celestial_body_mean_radius_m_[index];
  }

  // Parameters
  /**
   * @fn GetNumberOfSelectedBodies
   * @brief Return number of selected body
   */
  inline int GetNumberOfSelectedBodies(void) const { return number_of_selected_bodies_; }
  /**
   * @fn GetSelectedBody
   * @brief Return SPICE IDs of selected bodies
   */
  inline const int* GetSelectedBodyIds(void) const { return selected_body_ids_; }
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
  inline EarthRotation GetEarthRotation(void) const { return *earth_rotation_; };

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
  unsigned int number_of_selected_bodies_;     //!< Number of selected body
  int* selected_body_ids_;                     //!< SPICE IDs of selected bodies
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
  EarthRotation* earth_rotation_;  //!< Instance of Earth rotation
  RotationMode rotation_mode_;         //!< Designation of rotation model

  /**
   * @fn GetPlanetOrbit
   * @brief Get position/velocity of planet.
   * @note This is an override function of SPICE's spkezr_c (https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/cspice/spkezr_c.html)
   * @param [in] planet_name: Nama of planet defined by SPICE
   * @param [in] et: Ephemeris time
   * @param [out] orbit: Cartesian state vector representing the position and velocity of the target body relative to the specified observer.
   */
  void GetPlanetOrbit(const char* planet_name, const double et, double orbit[6]);
};

/**
 *@fn InitCelestialInfo
 *@brief Initialize function for CelestialInformation class
 *@param [in] file_name: Path to the initialize function
 */
CelestialInformation* InitCelestialInformation(std::string file_name);

#endif  // S2E_ENVIRONMENT_GLOBAL_CELESTIAL_INFORMATION_HPP_
