/**
 * @file geodetic_position.hpp
 * @brief Class to mange geodetic position expression
 */

#ifndef S2E_LIBRARY_GEODESY_GEODETIC_POSITION_HPP_
#define S2E_LIBRARY_GEODESY_GEODETIC_POSITION_HPP_

#include <math_physics/math/quaternion.hpp>
#include <math_physics/math/vector.hpp>

namespace geodesy {

/**
 * @class GeodeticPosition
 * @brief Class to mange geodetic position expression
 */
class GeodeticPosition {
 public:
  /**
   * @fn GeodeticPosition
   * @brief Default constructor
   */
  GeodeticPosition();
  /**
   * @fn GeodeticPosition
   * @brief Constructor with initialize
   * @param [in] latitude_rad: Latitude [rad]
   * @param [in] longitude_rad: Longitude [rad]
   * @param [in] altitude_m: Altitude [m]
   */
  GeodeticPosition(const double latitude_rad, const double longitude_rad, const double altitude_m);

  /**
   * @fn UpdateFromEcef
   * @brief Update geodetic position with position vector in the ECEF frame
   * @param [in] position_ecef_m: Position vector in the ECEF frame [m]
   */
  void UpdateFromEcef(const math::Vector<3> position_ecef_m);

  /**
   * @fn CalcEcefPosition
   * @brief Calculate and return the ECEF position from the geodetic position
   */
  math::Vector<3> CalcEcefPosition() const;

  // Getter
  /**
   * @fn GetLatitude_rad
   * @brief Return latitude [rad]
   */
  inline double GetLatitude_rad() const { return latitude_rad_; }
  /**
   * @fn GetLongitude_rad
   * @brief Return longitude [rad]
   */
  inline double GetLongitude_rad() const { return longitude_rad_; }
  /**
   * @fn GetAltitude_m
   * @brief Return altitude [m]
   */
  inline double GetAltitude_m() const { return altitude_m_; }
  /**
   * @fn GetQuaternionXcxfToLtc
   * @brief Conversion quaternion from XCXF (e.g. ECEF) to LTC frame
   */
  inline math::Quaternion GetQuaternionXcxfToLtc() const { return quaternion_xcxf_to_ltc_; }

 private:
  double latitude_rad_;   //!< Latitude [rad] South: -π/2 to 0, North: 0 to π/2
  double longitude_rad_;  //!< Longitude [rad] East: 0 to π, West: 2π to π (i.e., defined as 0 to 2π [rad] east of the Greenwich meridian)
  double altitude_m_;     //!< Altitude [m]

  math::Quaternion quaternion_xcxf_to_ltc_;  //!< Conversion quaternion from XCXF (e.g. ECEF) to LTC (Local Topographic Coordinate)

  /**
   * @fn CalcQuaternionXcxfToLtc
   * @brief Calculate quaternion which converts XCXF frame to LTC frame at the geodetic position
   */
  void CalcQuaternionXcxfToLtc();
};

}  // namespace geodesy

#endif  // S2E_LIBRARY_GEODESY_GEODETIC_POSITION_HPP_