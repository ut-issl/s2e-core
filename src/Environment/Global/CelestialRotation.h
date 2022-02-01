#ifndef __celestial_rotation_H__
#define __celestial_rotation_H__

#include <cstring>
#include <string>

#include <Library/math/Matrix.hpp>
#include <Library/math/Vector.hpp>
#include <Library/math/MatVec.hpp>
#include <Library/math/Quaternion.hpp>
#include <Interface/LogOutput/ILoggable.h>

using libra::Vector;
using libra::Quaternion;

enum RotationMode
{
    Idle,
    Simple,
    Full,
};

// class for describing the rotational motion of center objects，instantiated as private members of CelesInfo（ひとまず地球自転のみに対応）
// アルゴリズムは暫定的に，福島,"天体の回転運動理論入門講義ノート",2007, 及び，長沢,"天体の位置計算(増補版)",2001に従う.(IERS Conventions 2003)
class CelestialRotation
{
public:
  // initialize DCM to unit matrix in the default constructor
  CelestialRotation(const RotationMode rotation_mode, const std::string center_obj);
  // calculate rotation
  void Update(const double JulianDate);
  // get the DCM between J2000 and the coordinate system attached to the surface of the target object X (X-Centered X-Fixed)
  inline const Matrix<3, 3> GetDCMJ2000toXCXF() const { return DCM_J2000toXCXF_; };
  // get the DCM between TEME (Inertial coordinate used in SGP4) and the coordinate system attached to the surface of the target object X (X-Centered X-Fixed)
  inline const Matrix<3, 3> GetDCMTEMEtoXCXF() const { return DCM_TEMEtoXCXF_; };

private:
  // coefficient initialization function, 対象天体ごとに用意するか…？
  void Init_CelestialRotation_As_Earth(const RotationMode rotation_mode, const std::string center_obj);
  Matrix<3, 3> AxialRotation(const double GAST_rad);          // movement of the coordinate axes due to rotation around the rotation axis
  Matrix<3, 3> Nutation(const double (&tTT_century)[4]);      // movement of the coordinate axes due to Nutation
  Matrix<3, 3> Precession(const double (&tTT_century)[4]);    // movement of the coordinate axes due to Precession
  Matrix<3, 3> PolarMotion(const double Xp, const double Yp); // movement of the coordinate axes due to Polar Motion

  double dpsi_rad_;              // nutation in obliquity [rad]
  double depsilon_rad_;          // nutation in longitude [rad]
  double epsi_rad_;              // mean obliquity of the ecliptic [rad]
  Matrix<3, 3> DCM_J2000toXCXF_; // DCM J2000 to XCXF(X-Centered X-Fixed)
  Matrix<3, 3> DCM_TEMEtoXCXF_;  // DCM TEME to XCXF(X-Centered X-Fixed)
  RotationMode rotation_mode_;   // designation of dynamics model, "Idle":no motion，"Simple":rotation only，"Full":full-dynamics
  std::string planet_name_;           // designate which solar planet the instance should work as 

  // definitions of coefficeints(地球以外の天体の場合はケアすべき天体が変わりうる気もするが，ひとまず日月歳差の形式を前提とする)
  // (実装時の理解では)頻繁に変わるものではなさそうなので暫定的に初期化関数内でべた書きで初期化
  // 将来的には設定ファイル読込にすることも視野に入れる

  // coefficients for computing mean obliquity of the ecliptic
  double c_epsi_rad_[4];

  // coefficients for computing five delauney angles(l=lm,l'=ls,F,D,Ω=O) 
  double c_lm_rad_[5];
  double c_ls_rad_[5];
  double c_F_rad_[5];
  double c_D_rad_[5];
  double c_O_rad_[5];

  // coefficients for computing nutation angles(delta-epsilon, delta-psi)
  double c_depsilon_rad_[9];
  double c_dpsi_rad_[9];

  // coefficients for computing precession angles(zeta, theta, z)
  double c_zeta_rad_[3];
  double c_theta_rad_[3];
  double c_z_rad_[2];

  // time difference b/w UT1 and UTC [sec]
  const double dtUT1UTC_ = 32.184;

  // definitions of constant names
  const double kSec2Day = 1/(24.0 * 60.0 * 60.0);
  const double kJulianDateJ2000 = 2451545.0; // [day]
  const double kDayJulianCentury = 36525;    // [day/century]
};

#endif //__celestial_rotation_H__
