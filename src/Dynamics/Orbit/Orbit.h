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

#include <Environment/Global/PhysicalConstants.hpp>
#include <Library/Geodesy/GeodeticPosition.hpp>

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
  inline GeodeticPosition GetGeodeticPosition() const { return geo_pos_; }

  // TODO delete somedays...
  inline double GetLat_rad() const { return geo_pos_.GetLat_rad(); }
  inline double GetLon_rad() const { return geo_pos_.GetLon_rad(); }
  inline double GetAlt_m() const { return geo_pos_.GetAlt_m(); }
  inline Vector<3> GetLatLonAlt() const {
    Vector<3> vec;
    vec(0) = geo_pos_.GetLat_rad();
    vec(1) = geo_pos_.GetLon_rad();
    vec(2) = geo_pos_.GetAlt_m();
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

  Vector<3> sat_position_i_;     //! spacecraft position in inertial frame [m]
  Vector<3> sat_position_ecef_;  //! spacecraft position in ECEF frame [m]
  GeodeticPosition geo_pos_;     //! spacecraft position in Geodetic frame

  Vector<3> sat_velocity_i_;     //! spacecraft velocity in inertial frame [m/s]
  Vector<3> sat_velocity_b_;     //! spacecraft velocity in body frame [m/s]
  Vector<3> sat_velocity_ecef_;  //! spacecraft velocity in ECEF frame [m/s]

  // spacecraft acceleration in inertial frame [m/s2]
  // NOTE: Clear to zero at the end of the Propagate function
  Vector<3> acc_i_;

  // Frame Conversion TODO: consider other planet
  void TransEciToEcef(void);
  void TransEcefToGeo(void);
};

#endif  //__orbit_H__
