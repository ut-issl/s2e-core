/**
 * @file GroundStation.h
 * @brief Base class of ground station
 */

#pragma once

#include <Environment/Global/CelestialRotation.h>
#include <Simulation/Spacecraft/Spacecraft.h>

#include <Library/Geodesy/GeodeticPosition.hpp>
#include <Library/math/Vector.hpp>

#include "../SimulationConfig.h"

/**
 * @class GroundStation
 * @brief Base class of ground station
 */
class GroundStation : public ILoggable {
 public:
  /**
   * @fn GroundStation
   * @brief Constructor
   */
  GroundStation(SimulationConfig* config, int gs_id_);
  /**
   * @fn ~GroundStation
   * @brief Deconstructor
   */
  virtual ~GroundStation();

  /**
   * @fn Initialize
   * @brief Virtual function to initialize the ground station
   */
  virtual void Initialize(int gs_id, SimulationConfig* config);
  /**
   * @fn LogSetup
   * @brief Virtual function to log output setting for ground station related components
   */
  virtual void LogSetup(Logger& logger);
  /**
   * @fn Update
   * @brief Virtual function of main routine
   */
  virtual void Update(const CelestialRotation& celes_rotation, const Spacecraft& spacecraft);

  // Override functions for ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override function of log header setting
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override function of log value setting
   */
  virtual std::string GetLogValue() const;

  // Getters
  /**
   * @fn GetGsId
   * @brief Return ground station ID
   */
  int GetGsId() const { return gs_id_; }
  /**
   * @fn GetGSPosition_geo
   * @brief Return ground station position in the geodetic frame
   */
  GeodeticPosition GetGSPosition_geo() const { return gs_position_geo_; }
  /**
   * @fn GetGSPosition_ecef
   * @brief Return ground station position in the ECEF frame [m]
   */
  Vector<3> GetGSPosition_ecef() const { return gs_position_ecef_; }
  /**
   * @fn GetGSPosition_i
   * @brief Return ground station position in the inertial frame [m]
   */
  Vector<3> GetGSPosition_i() const { return gs_position_i_; }
  /**
   * @fn GetElevationLimitAngle_deg
   * @brief Return ground station elevation limit angle [deg]
   */
  double GetElevationLimitAngle_deg() const { return elevation_limit_angle_deg_; }
  /**
   * @fn IsVisible
   * @brief Return visible flag for the target spacecraft
   * @param [in] sc_id: target spacecraft ID
   */
  bool IsVisible(const int sc_id) const { return is_visible_.at(sc_id); }

 protected:
  int gs_id_;                         //!< Ground station ID
  GeodeticPosition gs_position_geo_;  //!< Ground Station Position in the geodetic frame
  Vector<3> gs_position_ecef_;        //!< Ground Station Position in the ECEF frame [m]
  Vector<3> gs_position_i_;           //!< Ground Station Position in the inertial frame [m]
  double elevation_limit_angle_deg_;  //!< Minimum elevation angle to work the ground station [deg]

  std::map<int, bool> is_visible_;  //!< Visible flag for each spacecraft ID (not care antenna)
  int num_sc_;                      //!< Number of spacecraft in the simulation

  // Return true when the satellite is visible from the ground station
  /**
   * @fn GetElevationLimitAngle_deg
   * @brief Calculate the visibility for the target spacecraft
   * @param [in] sc_pos_ecef_m: spacecraft position in ECEF frame [m]
   * @return True when the satellite is visible from the ground station
   */
  bool CalcIsVisible(const Vector<3> sc_pos_ecef_m);
};
