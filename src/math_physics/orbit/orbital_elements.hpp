/**
 * @file orbital_elements.hpp
 * @brief Class for classical orbital elements
 */

#ifndef S2E_LIBRARY_ORBIT_ORBITAL_ELEMENTS_HPP_
#define S2E_LIBRARY_ORBIT_ORBITAL_ELEMENTS_HPP_

#include "../math/vector.hpp"

namespace orbit {

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
   * @param[in] gravity_constant_m3_s2: Gravity constant [m3/s2]
   * @param[in] time_jday: Time expressed as Julian day
   * @param[in] position_i_m: Position vector in the inertial frame [m]
   * @param[in] velocity_i_m_s: Velocity vector in the inertial frame [m/s]
   */
  OrbitalElements(const double gravity_constant_m3_s2, const double time_jday, const math::Vector<3> position_i_m,
                  const math::Vector<3> velocity_i_m_s);
  /**
   * @fn ~OrbitalElements
   * @brief Destructor
   */
  ~OrbitalElements();

  // Getter
  /**
   * @fn GetSemiMajorAxis_m
   * @brief Return semi major axis [m]
   */
  inline double GetSemiMajorAxis_m() const { return semi_major_axis_m_; }
  /**
   * @fn GetEccentricity
   * @brief Return eccentricity
   */
  inline double GetEccentricity() const { return eccentricity_; }
  /**
   * @fn GetInclination_rad
   * @brief Return inclination [rad]
   */
  inline double GetInclination_rad() const { return inclination_rad_; }
  /**
   * @fn GetRaan_rad
   * @brief Return Right Ascension of the Ascending Node [rad]
   */
  inline double GetRaan_rad() const { return raan_rad_; }
  /**
   * @fn GetArgPerigee_rad
   * @brief Return argument of Perigee [rad]
   */
  inline double GetArgPerigee_rad() const { return arg_perigee_rad_; }
  /**
   * @fn GetEpoch_jday
   * @brief Return epoch (time at the perigee) [julian day]
   */
  inline double GetEpoch_jday() const { return epoch_jday_; }

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
  /**
   * @fn CalcOeFromPosVel
   * @brief Calculation of Orbital Elements from Position and Velocity
   * @param[in] gravity_constant_m3_s2: Gravity Constant of the center body
   * @param[in] time_jday: Time expressed as Julian day
   * @param[in] position_i_m: Position vector in the inertial frame [m]
   * @param[in] velocity_i_m_s: Velocity vector in the inertial frame [m/s]
   */
  void CalcOeFromPosVel(const double gravity_constant_m3_s2, const double time_jday, const math::Vector<3> position_i_m,
                        const math::Vector<3> velocity_i_m_s);
};

}  // namespace orbit

#endif  // S2E_LIBRARY_ORBIT_ORBITAL_ELEMENTS_HPP_
