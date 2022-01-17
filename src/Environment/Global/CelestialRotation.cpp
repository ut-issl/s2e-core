#include <iostream>
#include <sstream>

#include "../../Library/sgp4/sgp4ext.h"  //for jday()
#include "../../Library/sgp4/sgp4unit.h" //for gstime()
#include "CelestialRotation.h"

using namespace std;

#define DEG2RAD 0.017453292519943295769 // PI/180
#define ASEC2RAD DEG2RAD / 3600.0

// default constructor
CelestialRotation::CelestialRotation(const RotationMode rotation_mode,
                                     const string center_obj) {
  planet_name_ = "Anonymous";
  rotation_mode_ = Idle;
  unitalize(DCM_J2000toXCXF_);
  DCM_TEMEtoXCXF_ = DCM_J2000toXCXF_;
  if (center_obj == "EARTH") {
    Init_CelestialRotation_As_Earth(rotation_mode, center_obj);
  }
};

// initialize the class CelestialRotation instance as Earth
void CelestialRotation::Init_CelestialRotation_As_Earth(
    const RotationMode rotation_mode, const string center_obj) {
  planet_name_ = "EARTH";
  if (center_obj == planet_name_) {
    if (rotation_mode == Simple) {
      rotation_mode_ = Simple;

      // 自転軸周りの回転だけなら，係数初期化は不要か
    } else if (rotation_mode == Full) {
      rotation_mode_ = Full;

      // フルダイナミクスを使う場合は，ひとまず直接，各種係数の初期化を実施
      // 文献との比較確認がし易い様に，単位系は(敢えて)文献に良くあるdeg,arcsec混在系で定義しておく
      // coefficients for computing mean obliquity of the ecliptic
      // the actual unit of c_epsi_rad_ is [rad/century^i], i is the index of
      // the array
      c_epsi_rad_[0] = 23.4392911 * DEG2RAD;   // [rad]
      c_epsi_rad_[1] = -46.8150000 * ASEC2RAD; // [rad/century]
      c_epsi_rad_[2] = -5.9000e-4 * ASEC2RAD;  // [rad/century^2]
      c_epsi_rad_[3] = 1.8130e-3 * ASEC2RAD;   // [rad/century^3]

      // coefficients for computing five delauney angles(l=lm,l'=ls,F,D,Ω=O)
      // lm means mean anomaly of the moon
      // the actual unit of c_lm_rad_ is [rad/century^i], i is the index of the
      // array
      c_lm_rad_[0] = 134.96340251 * DEG2RAD;         // [rad]
      c_lm_rad_[1] = 1717915923.21780000 * ASEC2RAD; // [rad/century]
      c_lm_rad_[2] = 31.87920000 * ASEC2RAD;         // [rad/century^2]
      c_lm_rad_[3] = 0.05163500 * ASEC2RAD;          // [rad/century^3]
      c_lm_rad_[4] = -0.00024470 * ASEC2RAD;         // [rad/century^4]
      // ls means mean anomaly of the sun
      // the actual unit of c_ls_rad_ is [rad/century^i], i is the index of the
      // array
      c_ls_rad_[0] = 357.52910918 * DEG2RAD;        // [rad]
      c_ls_rad_[1] = 129596581.04810000 * ASEC2RAD; // [rad/century]
      c_ls_rad_[2] = -0.55320000 * ASEC2RAD;        // [rad/century^2]
      c_ls_rad_[3] = 0.00013600 * ASEC2RAD;         // [rad/century^3]
      c_ls_rad_[4] = -0.00001149 * ASEC2RAD;        // [rad/century^4]
      // F means mean longitude of the moon - mean longitude of ascending node
      // of the moon the actual unit of c_F_rad_ is [rad/century^i], i is the
      // index of the array
      c_F_rad_[0] = 93.27209062 * DEG2RAD;          // [rad]
      c_F_rad_[1] = 1739527262.84780000 * ASEC2RAD; // [rad/century]
      c_F_rad_[2] = -12.75120000 * ASEC2RAD;        // [rad/century^2]
      c_F_rad_[3] = -0.00103700 * ASEC2RAD;         // [rad/century^3]
      c_F_rad_[4] = 0.00000417 * ASEC2RAD;          // [rad/century^4]
      // D means mean elogation of the moon from the sun
      // the actual unit of c_D_rad_ is [rad/century^i], i is the index of the
      // array
      c_D_rad_[0] = 297.85019547 * DEG2RAD;         // [rad]
      c_D_rad_[1] = 1602961601.20900000 * ASEC2RAD; // [rad/century]
      c_D_rad_[2] = -6.37060000 * ASEC2RAD;         // [rad/century^2]
      c_D_rad_[3] = 0.00659300 * ASEC2RAD;          // [rad/century^3]
      c_D_rad_[4] = -0.00003169 * ASEC2RAD;         // [rad/century^4]
      // O means mean longitude of ascending node of the moon
      // the actual unit of c_O_rad_ is [rad/century^i], i is the index of the
      // array
      c_O_rad_[0] = 125.04455501 * DEG2RAD;       // [rad]
      c_O_rad_[1] = -6962890.54310000 * ASEC2RAD; // [rad/century]
      c_O_rad_[2] = 7.47220000 * ASEC2RAD;        // [rad/century^2]
      c_O_rad_[3] = 0.00770200 * ASEC2RAD;        // [rad/century^3]
      c_O_rad_[4] = -0.00005939 * ASEC2RAD;       // [rad/century^4]

      // coefficients for computing nutation angles
      // delta epsilon
      c_depsilon_rad_[0] = 9.2050 * ASEC2RAD;  // [rad]
      c_depsilon_rad_[1] = 0.5730 * ASEC2RAD;  // [rad]
      c_depsilon_rad_[2] = -0.0900 * ASEC2RAD; // [rad]
      c_depsilon_rad_[3] = 0.0980 * ASEC2RAD;  // [rad]
      c_depsilon_rad_[4] = 0.0070 * ASEC2RAD;  // [rad]
      c_depsilon_rad_[5] = -0.0010 * ASEC2RAD; // [rad]
      c_depsilon_rad_[6] = 0.0220 * ASEC2RAD;  // [rad]
      c_depsilon_rad_[7] = 0.0130 * ASEC2RAD;  // [rad]
      c_depsilon_rad_[8] = -0.0100 * ASEC2RAD; // [rad]
      // delta psi
      c_dpsi_rad_[0] = -17.2060 * ASEC2RAD; // [rad]
      c_dpsi_rad_[1] = -1.3170 * ASEC2RAD;  // [rad]
      c_dpsi_rad_[2] = 0.2070 * ASEC2RAD;   // [rad]
      c_dpsi_rad_[3] = -0.2280 * ASEC2RAD;  // [rad]
      c_dpsi_rad_[4] = 0.1480 * ASEC2RAD;   // [rad]
      c_dpsi_rad_[5] = 0.0710 * ASEC2RAD;   // [rad]
      c_dpsi_rad_[6] = -0.0520 * ASEC2RAD;  // [rad]
      c_dpsi_rad_[7] = -0.0300 * ASEC2RAD;  // [rad]
      c_dpsi_rad_[8] = 0.0220 * ASEC2RAD;   // [rad]

      // coefficients for computing precession angles(zeta, theta, z)
      // the actual unit of c_zeta_rad_ is [rad/century^(i+1)], i is the index
      // of the array
      c_zeta_rad_[0] = 2306.218100 * ASEC2RAD; // [rad/century]
      c_zeta_rad_[1] = 0.301880 * ASEC2RAD;    // [rad/century^2]
      c_zeta_rad_[2] = 0.017998 * ASEC2RAD;    // [rad/century^3]
      // the actual unit of c_theta_rad_ is [rad/century^(i+1)], i is the index
      // of the array
      c_theta_rad_[0] = 2004.310900 * ASEC2RAD; // [rad/century]
      c_theta_rad_[1] = -0.426650 * ASEC2RAD;   // [rad/century^2]
      c_theta_rad_[2] = -0.041833 * ASEC2RAD;   // [rad/century^3]
      // the actual unit of c_z_rad_ is [rad/century^(i+1)], i is the index of
      // the array
      c_z_rad_[0] = 2306.218100 * ASEC2RAD; // [rad/century]
      c_z_rad_[1] = 1.094680 * ASEC2RAD;    // [rad/century^2]
      c_z_rad_[2] = 0.018203 * ASEC2RAD;    // [rad/century^3]
    } else {
      // if the rotation mode is neither Simple nor Full, disable the rotation
      // calculation and make the DCM a unit matrix
      rotation_mode_ = Idle;
      unitalize(DCM_J2000toXCXF_);
    }
  } else {
    // if the center object is not the Earth, disable the Earth's rotation
    // calculation and make the DCM a unit matrix
    rotation_mode_ = Idle;
    unitalize(DCM_J2000toXCXF_);
  }
}

void CelestialRotation::Update(const double JulianDate) {
  double gmst_rad = gstime(
      JulianDate); //長沢のアルゴリズムと微妙に違う…？，他と合わせた方が良いので，暫定このままで行く．

  if (rotation_mode_ == Full) {
    // compute Julian date for terestrial time
    double jdTT_day =
        JulianDate +
        dtUT1UTC_ *
            kSec2Day; // 不正確かも(?)だが，S2E内部でグレゴリオ暦の伝播はしていないので，これで済ませる

    // compute nth power of julian century for terrestrial time
    // the actual unit of tTT_century is [century^(i+1)], i is the index of the
    // array
    double tTT_century[4];
    tTT_century[0] = (jdTT_day - kJulianDateJ2000) / kDayJulianCentury;
    for (int i = 0; i < 3; i++) {
      tTT_century[i + 1] = tTT_century[i] * tTT_century[0];
    }

    Matrix<3, 3> P;
    Matrix<3, 3> N;
    Matrix<3, 3> R;
    Matrix<3, 3> W;
    // Nutation + Precession
    P = Precession(tTT_century);
    N = Nutation(tTT_century); // epsi_rad_, depsilon_rad_, dpsi_rad_ are
                               // updated in this proccedure

    // Axial Rotation
    double Eq_rad =
        dpsi_rad_ *
        cos(epsi_rad_ + depsilon_rad_); // equation of equinoxes [rad]
    double gast_rad =
        gmst_rad + Eq_rad; // Greenwitch 'Appearent' Sidereal Time [rad]
    R = AxialRotation(gast_rad);
    // polar motion (isnot considered so far, even without polar motion, the
    // result agrees well with the matlab reference)
    double Xp = 0.0;
    double Yp = 0.0;
    W = PolarMotion(Xp, Yp);

    // total orientation
    DCM_J2000toXCXF_ = W * R * N * P;
  } else if (rotation_mode_ == Simple) {
    // In this case, only Axial Rotation is executed, with its argument replaced
    // from G'A'ST to G'M'ST
    DCM_J2000toXCXF_ = AxialRotation(gmst_rad);
  } else {
    // leave the DCM as unit Matrix(diag{1,1,1})
    return;
  }
}

Matrix<3, 3> CelestialRotation::AxialRotation(const double GAST_rad) {
  return libra::rotz(GAST_rad);
}

Matrix<3, 3> CelestialRotation::Nutation(const double (&tTT_century)[4]) {
  // mean obliquity of the ecliptic
  epsi_rad_ = c_epsi_rad_[0]; // [rad]
  for (int i = 0; i < 3; i++) {
    epsi_rad_ += c_epsi_rad_[i + 1] * tTT_century[i];
  }

  // compute five delauney angles(l=lm,l'=ls,F,D,Ω=O)
  // mean anomaly of the moon
  double lm_rad = c_lm_rad_[0]; // [rad]
  for (int i = 0; i < 4; i++) {
    lm_rad += c_lm_rad_[i + 1] * tTT_century[i];
  }
  // mean anomaly of the sun
  double ls_rad = c_ls_rad_[0]; // [rad]
  for (int i = 0; i < 4; i++) {
    ls_rad += c_ls_rad_[i + 1] * tTT_century[i];
  }
  // mean longitude of the moon - mean longitude of ascending node of the moon
  double F_rad = c_F_rad_[0]; // [rad]
  for (int i = 0; i < 4; i++) {
    F_rad += c_F_rad_[i + 1] * tTT_century[i];
  }
  // mean elogation of the moon from the sun
  double D_rad = c_D_rad_[0]; // [rad]
  for (int i = 0; i < 4; i++) {
    D_rad += c_D_rad_[i + 1] * tTT_century[i];
  }
  // mean longitude of ascending node of the moon
  double O_rad = c_O_rad_[0]; // [rad]
  for (int i = 0; i < 4; i++) {
    O_rad += c_O_rad_[i + 1] * tTT_century[i];
  }

  // additional angles
  double L_rad = F_rad + O_rad;  // F + Ω     [rad]
  double Ld_rad = L_rad - D_rad; // F - D + Ω [rad]

  // compute luni-solar nutation
  // nutation in obliquity
  dpsi_rad_ = c_dpsi_rad_[0] * sin(O_rad) + c_dpsi_rad_[1] * sin(2 * Ld_rad) +
              c_dpsi_rad_[2] * sin(2 * O_rad) +
              c_dpsi_rad_[3] * sin(2 * L_rad) +
              c_dpsi_rad_[4] * sin(ls_rad); // [rad]
  dpsi_rad_ = dpsi_rad_ + c_dpsi_rad_[5] * sin(lm_rad) +
              c_dpsi_rad_[6] * sin(2 * Ld_rad + ls_rad) +
              c_dpsi_rad_[7] * sin(2 * L_rad + lm_rad) +
              c_dpsi_rad_[8] * sin(2 * Ld_rad - ls_rad); // [rad]

  // nutation in longitude
  depsilon_rad_ = c_depsilon_rad_[0] * cos(O_rad) +
                  c_depsilon_rad_[1] * cos(2 * Ld_rad) +
                  c_depsilon_rad_[2] * cos(2 * O_rad) +
                  c_depsilon_rad_[3] * cos(2 * L_rad) +
                  c_depsilon_rad_[4] * cos(ls_rad); // [rad]
  depsilon_rad_ = depsilon_rad_ + c_depsilon_rad_[5] * cos(lm_rad) +
                  c_depsilon_rad_[6] * cos(2 * Ld_rad + ls_rad) +
                  c_depsilon_rad_[7] * cos(2 * L_rad + lm_rad) +
                  c_depsilon_rad_[8] * cos(2 * Ld_rad - ls_rad); // [rad]

  double epsi_mod_rad = epsi_rad_ + depsilon_rad_;
  Matrix<3, 3> X_epsi_1st = libra::rotx(epsi_rad_);
  Matrix<3, 3> Z_dpsi = libra::rotz(-dpsi_rad_);
  Matrix<3, 3> X_epsi_2nd = libra::rotx(-epsi_mod_rad);

  Matrix<3, 3> N;
  N = X_epsi_2nd * Z_dpsi * X_epsi_1st;

  return N;
}

Matrix<3, 3> CelestialRotation::Precession(const double (&tTT_century)[4]) {
  // compute precession angles(zeta, theta, z)
  double zeta_rad = 0.0; // [rad]
  for (int i = 0; i < 3; i++) {
    zeta_rad += c_zeta_rad_[i] * tTT_century[i];
  }
  double theta_rad = 0.0; // [rad]
  for (int i = 0; i < 3; i++) {
    theta_rad += c_theta_rad_[i] * tTT_century[i];
  }
  double z_rad = 0.0; // [rad]
  for (int i = 0; i < 3; i++) {
    z_rad += c_z_rad_[i] * tTT_century[i];
  }

  // develop transformation matrix
  Matrix<3, 3> Z_zeta = libra::rotz(-zeta_rad);
  Matrix<3, 3> Y_theta = libra::roty(theta_rad);
  Matrix<3, 3> Z_z = libra::rotz(-z_rad);

  Matrix<3, 3> P;
  P = Z_z * Y_theta * Z_zeta;

  return P;
}

Matrix<3, 3> CelestialRotation::PolarMotion(const double Xp, const double Yp) {
  Matrix<3, 3> W;

  W[0][0] = 1.0;
  W[0][1] = 0.0;
  W[0][2] = -Xp;
  W[1][0] = 0.0;
  W[1][1] = 1.0;
  W[1][2] = -Yp;
  W[2][0] = Xp;
  W[2][1] = Yp;
  W[2][2] = 1.0;

  return W;
}