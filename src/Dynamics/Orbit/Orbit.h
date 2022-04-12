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

#include <Environment/Global/CelestialInformation.h>
#include <Interface/LogOutput/ILoggable.h>
#include <math.h>

#include <Environment/Global/PhysicalConstants.hpp>

// For ECI to GEO calculation
#include <Library/sgp4/sgp4ext.h>
#include <Library/sgp4/sgp4io.h>
#include <Library/sgp4/sgp4unit.h>

class Orbit : public ILoggable {
 public:
  Orbit(const CelestialInformation* celes_info) : celes_info_(celes_info) {}
  virtual ~Orbit() {}

  enum class PROPAGATE_MODE { RK4 = 0, SGP4, RELATIVE_ORBIT, KEPLER, ENCKE };

  // propagation of orbit
  virtual void Propagate(double endtime, double current_jd) = 0;

  // update of attitude information
  inline void UpdateAtt(Quaternion q_i2b) { sat_velocity_b_ = q_i2b.frame_conv(sat_velocity_i_); }

  // Shift the position of the spacecraft by the offset vector in the inertial frame
  inline virtual void AddPositionOffset(Vector<3> offset_i) { (void)offset_i; }

  // Getters
  inline bool GetIsCalcEnabled() const { return is_calc_enabled_; }
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

  // Setters
  inline void SetIsCalcEnabled(bool is_calc_enabled) { is_calc_enabled_ = is_calc_enabled; }
  inline void SetAcceleration_i(Vector<3> acceleration_i) { acc_i_ = acceleration_i; }
  inline void AddForce_i(Vector<3> force_i, double spacecraft_mass) {
    force_i /= spacecraft_mass;
    acc_i_ += force_i;
  }
  inline void AddAcceleration_i(Vector<3> acceleration_i) { acc_i_ += acceleration_i; }
  inline void AddForce_b(Vector<3> force_b, Quaternion q_i2b, double spacecraft_mass) {
    auto force_i = q_i2b.frame_conv_inv(force_b);
    AddForce_i(force_i, spacecraft_mass);
  }

  Quaternion CalcQuaternionI2LVLH() const;

  virtual std::string GetLogHeader() const = 0;
  virtual std::string GetLogValue() const = 0;

 protected:
  const CelestialInformation* celes_info_;

  // Settings
  bool is_calc_enabled_ = false;
  PROPAGATE_MODE propagate_mode_;
  gravconsttype whichconst;  // this value defines the constants in SGP4 calculation

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

  // Convert ECI satellite position to ECEF frame
  void TransECIToECEF(void);
  // Convert ECI satellite position to GEO(lattitude, longitude, and latitude)
  void TransECIToGeo(double current_jd);
};

#endif  //__orbit_H__
