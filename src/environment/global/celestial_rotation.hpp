/**
 * @file celestial_rotation.hpp
 * @brief Class to calculate the celestial rotation
 * @note Support earth rotation only now (TODO: add other planets)
 *       Refs: 福島,"天体の回転運動理論入門講義ノート", 2007 (in Japanese),
 *             長沢,"天体の位置計算(増補版)", 2001 (in Japanese),
 *             IERS Conventions 2003
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_CELESTIAL_ROTATION_H_
#define S2E_ENVIRONMENT_GLOBAL_CELESTIAL_ROTATION_H_

#include <interface/log_output/loggable.hpp>

#include <Library/math/MatVec.hpp>
#include <Library/math/Matrix.hpp>
#include <Library/math/Quaternion.hpp>
#include <Library/math/Vector.hpp>
#include <cstring>
#include <string>

using libra::Quaternion;
using libra::Vector;

/**
 * @enum RotationMode
 * @brief Definition of calculation mode of celestial rotation
 */
enum RotationMode {
  Idle,    //!< No Rotation calculation
  Simple,  //!< Z axis rotation only
  Full,    //!< Rotation including precession and nutation
};

/**
 * @class CelestialRotation
 * @brief Class to calculate the celestial rotation
 * @note Support earth rotation only now (TODO: add other planets)
 */
class CelestialRotation {
 public:
  // initialize DCM to unit matrix in the default constructor
  /**
   * @fn CelestialRotation
   * @brief Constructor
   * @param [in] rotation_mode: Designation of rotation model
   * @param [in] center_obj: Center object of inertial frame
   */
  CelestialRotation(const RotationMode rotation_mode, const std::string center_obj);

  /**
   * @fn Update
   * @brief Update rotation
   * @param [in] JulianDate: Julian date
   */
  void Update(const double JulianDate);

  /**
   * @fn GetDCMJ2000toXCXF
   * @brief Return the DCM between J2000 inertial frame and the frame of fixed to the target object X (X-Centered X-Fixed)
   */
  inline const Matrix<3, 3> GetDCMJ2000toXCXF() const { return DCM_J2000toXCXF_; };

  /**
   * @fn GetDCMJ2000toXCXF
   * @brief Return the DCM between TEME (Inertial frame used in SGP4) and the frame of fixed to the target object X (X-Centered X-Fixed)
   */
  inline const Matrix<3, 3> GetDCMTEMEtoXCXF() const { return DCM_TEMEtoXCXF_; };

 private:
  /**
   * @fn Init_CelestialRotation_As_Earth
   * @brief Initialize CelestialRotation as earth rotation
   * @note TODO: Make functions for other planets?
   * @param [in] rotation_mode: Rotation mode
   * @param [in] center_obj: Name of center body
   */
  void Init_CelestialRotation_As_Earth(const RotationMode rotation_mode, const std::string center_obj);

  Matrix<3, 3> AxialRotation(const double GAST_rad);           //!< Movement of the coordinate axes due to rotation around the rotation axis
  Matrix<3, 3> Nutation(const double (&tTT_century)[4]);       //!< Movement of the coordinate axes due to Nutation
  Matrix<3, 3> Precession(const double (&tTT_century)[4]);     //!< Movement of the coordinate axes due to Precession
  Matrix<3, 3> PolarMotion(const double Xp, const double Yp);  //!< Movement of the coordinate axes due to Polar Motion

  double dpsi_rad_;               //!< Nutation in obliquity [rad]
  double depsilon_rad_;           //!< Nutation in longitude [rad]
  double epsi_rad_;               //!< Mean obliquity of the ecliptic [rad]
  Matrix<3, 3> DCM_J2000toXCXF_;  //!< Direction Cosine Matrix J2000 to XCXF(X-Centered X-Fixed)
  Matrix<3, 3> DCM_TEMEtoXCXF_;   //!< Direction Cosine Matrix TEME to XCXF(X-Centered X-Fixed)
  RotationMode rotation_mode_;    //!< Designation of dynamics model
  std::string planet_name_;       //!< Designate which solar planet the instance should work as

  // Definitions of coefficeints
  // They are handling as constant values
  // TODO: Consider to read setting files for these coefficients
  // TODO: Consider other formats for other planets
  double c_epsi_rad_[4];  //!< Coefficients to compute mean obliquity of the ecliptic
  double c_lm_rad_[5];    //!< Coefficients to compute delauney angle (l=lm: Mean anomaly of the moon)
  double c_ls_rad_[5];    //!< Coefficients to compute delauney angle (l'=ls: Mean anomaly of the sun)
  double c_F_rad_[5];     //!< Coefficients to compute delauney angle (F: Mean longitude of the moon - mean longitude of ascending node of the moon)
  double c_D_rad_[5];     //!< Coefficients to compute delauney angle (D: Elogation of the moon from the sun)
  double c_O_rad_[5];     //!< Coefficients to compute delauney angle (Ω=O: Mean longitude of ascending node of the moon)
  double c_depsilon_rad_[9];  //!< Coefficients to compute nutation angle (delta-epsilon)
  double c_dpsi_rad_[9];      //!< Coefficients to compute nutation angle (delta-psi)
  double c_zeta_rad_[3];      //!< Coefficients to compute precession angle (zeta)
  double c_theta_rad_[3];     //!< Coefficients to compute precession angle (theta)
  double c_z_rad_[2];         //!< Coefficients to compute precession angle (z)

  const double dtUT1UTC_ = 32.184;                   //!< Time difference b/w UT1 and UTC [sec]
  const double kSec2Day = 1 / (24.0 * 60.0 * 60.0);  //!< Conversion constant from sec to day
  const double kJulianDateJ2000 = 2451545.0;         //!< Julian date of J2000 [day]
  const double kDayJulianCentury = 36525;            //!< Conversion constant from Julian century to day [day/century]
};

#endif  // S2E_ENVIRONMENT_GLOBAL_CELESTIAL_ROTATION_H_
