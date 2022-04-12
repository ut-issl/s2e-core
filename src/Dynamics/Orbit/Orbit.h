#ifndef __orbit_H__
#define __orbit_H__

#include <Library/math/Constant.hpp>
#include <Library/math/MatVec.hpp>
#include <Library/math/Matrix.hpp>
#include <Library/math/Quaternion.hpp>
#include <Library/math/Vector.hpp>

using libra::Matrix;
using libra::Quaternion;
using libra::Vector;

#include <Interface/LogOutput/ILoggable.h>
#include <math.h>

#include <Environment/Global/PhysicalConstants.hpp>

// For ECI to GEO calculation
#include <Library/sgp4/sgp4ext.h>
#include <Library/sgp4/sgp4io.h>
#include <Library/sgp4/sgp4unit.h>

class Orbit : public ILoggable {
 public:
  enum class PROPAGATE_MODE { RK4 = 0, SGP4, RELATIVE_ORBIT, KEPLER, ENCKE };

  virtual ~Orbit() {}

  inline PROPAGATE_MODE GetPropagateMode() const { return propagate_mode_; }
  inline Vector<3> GetSatPosition_i() const { return sat_position_i_; }
  inline Vector<3> GetSatPosition_ecef() const { return sat_position_ecef_; }
  inline Vector<3> GetSatVelocity_i() const { return sat_velocity_i_; }
  inline Vector<3> GetSatVelocity_b() const { return sat_velocity_b_; }
  inline Vector<3> GetSatVelocity_ecef() const { return sat_velocity_ecef_; }
  inline virtual Vector<3> GetESIOmega() const { return Vector<3>(); }
  inline double GetLat_rad() const { return lat_rad_; }
  inline double GetLon_rad() const { return lon_rad_; }
  inline double GetAlt_m() const { return alt_m_; }
  inline Vector<3> GetLatLonAlt() const {
    Vector<3> vec;
    vec(0) = lat_rad_;
    vec(1) = lon_rad_;
    vec(2) = alt_m_;
    return vec;
  }

  // propagation of orbit
  virtual void Propagate(double endtime, double current_jd) = 0;

  // update of attitude information
  inline void UpdateAtt(Quaternion q_i2b) { sat_velocity_b_ = q_i2b.frame_conv(sat_velocity_i_); }

  inline void SetAcceleration_i(Vector<3> acceleration_i) { acc_i_ = acceleration_i; }

  // set force in an inertial frame
  inline void AddForce_i(Vector<3> force_i, double spacecraft_mass) {
    force_i /= spacecraft_mass;
    acc_i_ += force_i;
  }

  // set acceleration in inertial frame
  inline void AddAcceleration_i(Vector<3> acceleration_i) { acc_i_ += acceleration_i; }

  // set acceleration in body frame
  inline void AddForce_b(Vector<3> force_b, Quaternion q_i2b, double spacecraft_mass) {
    auto force_i = q_i2b.frame_conv_inv(force_b);
    AddForce_i(force_i, spacecraft_mass);
  }

  // Shift the position of the spacecraft by the offset vector in the inertial
  // frame
  inline virtual void AddPositionOffset(Vector<3> offset_i) { (void)offset_i; }

  Quaternion CalcQuaternionI2LVLH() const {
    Vector<3> lvlh_x = sat_position_i_;  // x-axis in LVLH frame is position vector direction
                                         // from geocenter to satellite
    Vector<3> lvlh_ex = normalize(lvlh_x);
    Vector<3> lvlh_z = outer_product(sat_position_i_,
                                     sat_velocity_i_);  // z-axis in LVLH frame is angular
                                                        // momentum vector direction of orbit
    Vector<3> lvlh_ez = normalize(lvlh_z);
    Vector<3> lvlh_y = outer_product(lvlh_z, lvlh_x);
    Vector<3> lvlh_ey = normalize(lvlh_y);

    Matrix<3, 3> dcm_i2lvlh;
    dcm_i2lvlh[0][0] = lvlh_ex[0];
    dcm_i2lvlh[0][1] = lvlh_ex[1];
    dcm_i2lvlh[0][2] = lvlh_ex[2];
    dcm_i2lvlh[1][0] = lvlh_ey[0];
    dcm_i2lvlh[1][1] = lvlh_ey[1];
    dcm_i2lvlh[1][2] = lvlh_ey[2];
    dcm_i2lvlh[2][0] = lvlh_ez[0];
    dcm_i2lvlh[2][1] = lvlh_ez[1];
    dcm_i2lvlh[2][2] = lvlh_ez[2];

    Quaternion q_i2lvlh = Quaternion::fromDCM(dcm_i2lvlh);
    return q_i2lvlh.normalize();
  }

  virtual std::string GetLogHeader() const = 0;
  virtual std::string GetLogValue() const = 0;

  bool IsCalcEnabled = false;
  gravconsttype whichconst;

 protected:
  // propagate mode
  PROPAGATE_MODE propagate_mode_;

  // spacecraft position in inertial frame [m]
  Vector<3> sat_position_i_;

  // spacecraft position in ECEF frame [m]
  Vector<3> sat_position_ecef_;

  // spacecraft velocity in inertial frame [m/s]
  Vector<3> sat_velocity_i_;

  // spacecraft velocity in body frame [m/s]
  Vector<3> sat_velocity_b_;

  // spacecraft velocity in ECEF frame [m/s]
  Vector<3> sat_velocity_ecef_;

  // spacecraft acceleration in inertial frame [m/s2]
  // NOTE: Clear to zero at the end of the Propagate function
  Vector<3> acc_i_;

  // Latitude [rad] South: -π/2 to 0, North: 0 to π/2
  double lat_rad_;

  // Longitude [rad] East longitude: 0 to π, West longitude: 2π to π (i.e.,
  // defined as 0 to 2π [rad] east of the Greenwich meridian)
  double lon_rad_;

  // Altitude [m]
  double alt_m_;

  // TransECItoECEF
  Matrix<3, 3> trans_eci2ecef_;

  // Convert ECI satellite position to ECEF frame
  inline void TransECIToECEF(double current_jd) {
    double current_side = gstime(current_jd);

    Matrix<3, 3> trans_mat;  // ECI2ECEF transformation matrix representing the Earth's
                             // rotation according to Greenwich Sidereal Time.
    trans_mat[0][0] = cos(current_side);
    trans_mat[0][1] = sin(current_side);
    trans_mat[0][2] = 0;
    trans_mat[1][0] = -sin(current_side);
    trans_mat[1][1] = cos(current_side);
    trans_mat[1][2] = 0;
    trans_mat[2][0] = 0;
    trans_mat[2][1] = 0;
    trans_mat[2][2] = 1;

    sat_position_ecef_ = trans_mat * sat_position_i_;

    // convert velocity vector in ECI to the vector in ECEF
    Vector<3> OmegaE{0.0};
    OmegaE[2] = environment::earth_mean_angular_velocity_rad_s;
    Vector<3> wExr = outer_product(OmegaE, sat_position_i_);
    Vector<3> V_wExr = sat_velocity_i_ - wExr;
    sat_velocity_ecef_ = trans_mat * V_wExr;

    trans_eci2ecef_ = trans_mat;
  }

  // Convert ECI satellite position to GEO(lattitude, longitude, and latitude)
  void TransECIToGeo(double current_jd) {
    double r, e2, phi, c;
    double theta;
    double current_side = gstime(current_jd);
    double radiusearthkm;
    double f;
    getwgsconst(whichconst, radiusearthkm, f);

    theta = AcTan(sat_position_i_[1], sat_position_i_[0]); /* radians */
    lon_rad_ = FMod2p(theta - current_side);               /* radians */
    r = sqrt(sat_position_i_[0] * sat_position_i_[0] + sat_position_i_[1] * sat_position_i_[1]);
    e2 = f * (2.0 - f);
    lat_rad_ = AcTan(sat_position_i_[2], r); /* radians */

    do {
      phi = lat_rad_;
      c = 1.0 / sqrt(1.0 - e2 * sin(phi) * sin(phi));
      lat_rad_ = AcTan(sat_position_i_[2] + radiusearthkm * c * e2 * sin(phi) * 1000.0, r);

    } while (fabs(lat_rad_ - phi) >= 1E-10);

    alt_m_ = r / cos(lat_rad_) - radiusearthkm * c * 1000.0;
    /* kilometers -> meters */  // Height of the ellipsoid

    if (lat_rad_ > libra::pi_2) lat_rad_ -= libra::tau;
  }
};

#endif  //__orbit_H__
