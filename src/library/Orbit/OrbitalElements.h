/**
 * @file OrbitalElements.h
 * @brief Class for classical orbital elements
 */

#pragma once
#include "../math/Vector.hpp"

/**
 * @class OrbitalElements
 * @brief Class for classical orbital elements
 */
class OrbitalElements {
 public:
  /**
   * @fn OrbitalElements
   * @brief Default Constructor
   */
  OrbitalElements();
  /**
   * @fn OrbitalElements
   * @brief Constructor: Initialize with OE
   * @param[in] epoch_jday: epoch (time at the perigee) [julian day]
   * @param[in] semi_major_axis_m: Semi major axis [m]
   * @param[in] eccentricity: Eccentricity
   * @param[in] inclination_rad: Inclination [rad]
   * @param[in] raan_rad: Right Ascension of the Ascending Node [rad]
   * @param[in] arg_perigee_rad: Argument of Perigee [rad]
   */
  OrbitalElements(const double epoch_jday, const double semi_major_axis_m, const double eccentricity, const double inclination_rad,
                  const double raan_rad, const double arg_perigee_rad);
  /**
   * @fn OrbitalElements
   * @brief Constructor: Initialize with position and velocity
   * @param[in] mu_m3_s2: Gravity constant [m3/s2]
   * @param[in] time_jday: Time expressed as Julian day
   * @param[in] position_i_m: Position vector in the inertial frame [m]
   * @param[in] velocity_i_m_s: Velocity vector in the inertial frame [m/s]
   */
  OrbitalElements(const double mu_m3_s2, const double time_jday, const libra::Vector<3> position_i_m, const libra::Vector<3> velocity_i_m_s);
  /**
   * @fn ~OrbitalElements
   * @brief Destructor
   */
  ~OrbitalElements();

  // Getter
  /**
   * @fn GetSemiMajor
   * @brief Return semi major axis [m]
   */
  inline double GetSemiMajor() const { return semi_major_axis_m_; }
  /**
   * @fn GetEccentricity
   * @brief Return eccentricity
   */
  inline double GetEccentricity() const { return eccentricity_; }
  /**
   * @fn GetInclination
   * @brief Return inclination [rad]
   */
  inline double GetInclination() const { return inclination_rad_; }
  /**
   * @fn GetRaan
   * @brief Return Right Ascension of the Ascending Node [rad]
   */
  inline double GetRaan() const { return raan_rad_; }
  /**
   * @fn GetArgPerigee
   * @brief Return argument of Perigee [rad]
   */
  inline double GetArgPerigee() const { return arg_perigee_rad_; }
  /**
   * @fn GetEpoch
   * @brief Return epoch (time at the perigee) [julian day]
   */
  inline double GetEpoch() const { return epoch_jday_; }

 private:
  // Common Variables
  // Shape
  double semi_major_axis_m_;  //!< Semi major axis [m]
  double eccentricity_;       //!< Eccentricity
  // Coordinate
  double inclination_rad_;  //!< Inclination [rad]
  double raan_rad_;         //!< Right Ascension of the Ascending Node [rad]
  double arg_perigee_rad_;  //!< Argument of Perigee [rad]
  // Time, phase
  double epoch_jday_;  //!< epoch (time at the perigee) [julian day]

  // Functions
  // Calculation of Orbital Elements from Position and Velocity
  void CalcOeFromPosVel(const double mu_m3_s2, const double time_jday, const libra::Vector<3> r_i_m, const libra::Vector<3> v_i_m_s);
};
