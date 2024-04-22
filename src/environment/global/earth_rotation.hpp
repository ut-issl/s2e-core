/**
 * @file earth_rotation.hpp
 * @brief Class to calculate the earth rotation
 * @note Refs: 福島,"天体の回転運動理論入門講義ノート", 2007 (in Japanese),
 *             長沢,"天体の位置計算(増補版)", 2001 (in Japanese),
 *             IERS Conventions 2003
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_EARTH_ROTATION_HPP_
#define S2E_ENVIRONMENT_GLOBAL_EARTH_ROTATION_HPP_

#include "math_physics/math/matrix.hpp"

/**
 * @enum EarthRotationMode
 * @brief Definition of calculation mode of earth rotation
 */
enum class EarthRotationMode {
  kIdle,    //!< No Rotation calculation
  kSimple,  //!< Z axis rotation only
  kFull,    //!< Rotation including precession and nutation
};

/**
 * @class EarthRotation
 * @brief Class to calculate the earth rotation
 */
class EarthRotation {
 public:
  /**
   * @fn EarthRotation
   * @brief Constructor
   * @param [in] rotation_mode: Designation of rotation model
   */
  EarthRotation(const EarthRotationMode rotation_mode = EarthRotationMode::kSimple);

  /**
   * @fn Update
   * @brief Update rotation
   * @param [in] julian_date: Julian date
   */
  void Update(const double julian_date);

  /**
   * @fn GetDcmJ2000ToEcef
   * @brief Return the DCM between J2000 inertial frame and the Earth Centered Earth Fixed frame
   */
  inline const libra::Matrix<3, 3> GetDcmJ2000ToEcef() const { return dcm_j2000_to_ecef_; };

  /**
   * @fn GetDcmTemeToEcef
   * @brief Return the DCM between TEME (Inertial frame used in SGP4) and the Earth Centered Earth Fixed frame
   */
  inline const libra::Matrix<3, 3> GetDcmTemeToEcef() const { return dcm_teme_to_ecef_; };

 private:
  double d_psi_rad_;                       //!< Nutation in obliquity [rad]
  double d_epsilon_rad_;                   //!< Nutation in longitude [rad]
  double epsilon_rad_;                     //!< Mean obliquity of the ecliptic [rad]
  libra::Matrix<3, 3> dcm_j2000_to_ecef_;  //!< Direction Cosine Matrix J2000 to ECEF
  libra::Matrix<3, 3> dcm_teme_to_ecef_;   //!< Direction Cosine Matrix TEME to ECEF
  EarthRotationMode rotation_mode_;        //!< Designation of dynamics model

  // Definitions of coefficients
  // They are handling as constant values
  // TODO: Consider to read setting files for these coefficients
  double c_epsilon_rad_[4];  //!< Coefficients to compute mean obliquity of the ecliptic
  double c_lm_rad_[5];       //!< Coefficients to compute Delaunay angle (l=lm: Mean anomaly of the moon)
  double c_ls_rad_[5];       //!< Coefficients to compute Delaunay angle (l'=ls: Mean anomaly of the sun)
  double c_f_rad_[5];  //!< Coefficients to compute Delaunay angle (F: Mean longitude of the moon - mean longitude of ascending node of the moon)
  double c_d_rad_[5];  //!< Coefficients to compute Delaunay angle (D: Elongation of the moon from the sun)
  double c_o_rad_[5];  //!< Coefficients to compute Delaunay angle (Ω=O: Mean longitude of ascending node of the moon)
  double c_d_epsilon_rad_[9];  //!< Coefficients to compute nutation angle (delta-epsilon)
  double c_d_psi_rad_[9];      //!< Coefficients to compute nutation angle (delta-psi)
  double c_zeta_rad_[3];       //!< Coefficients to compute precession angle (zeta)
  double c_theta_rad_[3];      //!< Coefficients to compute precession angle (theta)
  double c_z_rad_[3];          //!< Coefficients to compute precession angle (z)

  // TODO: Move to general constant values
  const double kDtUt1Utc_ = 32.184;                     //!< Time difference b/w UT1 and UTC [sec]
  const double kSec2Day_ = 1.0 / (24.0 * 60.0 * 60.0);  //!< Conversion constant from sec to day
  const double kJulianDateJ2000_ = 2451545.0;           //!< Julian date of J2000 [day]
  const double kDayJulianCentury_ = 36525.0;            //!< Conversion constant from Julian century to day [day/century]

  /**
   * @fn InitializeParameters
   * @brief Initialize parameters
   */
  void InitializeParameters();

  /**
   * @fn AxialRotation
   * @brief Calculate movement of the coordinate axes due to rotation around the rotation axis
   * @param [in] gast_rad: Greenwich 'Apparent' Sidereal Time [rad]
   * @return Rotation matrix
   */
  libra::Matrix<3, 3> AxialRotation(const double gast_rad);

  /**
   * @fn Nutation
   * @brief Calculate movement of the coordinate axes due to Nutation
   * @param [in] t_tt_century: nth power of julian century for terrestrial time
   * @return Rotation matrix
   */
  libra::Matrix<3, 3> Nutation(const double (&t_tt_century)[4]);

  /**
   * @fn Precession
   * @brief Calculate movement of the coordinate axes due to Precession
   * @param [in] t_tt_century: nth power of julian century for terrestrial time
   * @return Rotation matrix
   */
  libra::Matrix<3, 3> Precession(const double (&t_tt_century)[4]);

  /**
   * @fn PolarMotion
   * @brief Calculate movement of the coordinate axes due to Polar Motion
   * @note Currently, this function is not used.
   */
  libra::Matrix<3, 3> PolarMotion(const double x_p, const double y_p);
};

EarthRotationMode ConvertEarthRotationMode(const std::string mode);

#endif  // S2E_ENVIRONMENT_GLOBAL_EARTH_ROTATION_HPP_
