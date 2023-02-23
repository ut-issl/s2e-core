/**
 * @file orbit.hpp
 * @brief Base class of orbit propagation
 */

#ifndef S2E_DYNAMICS_ORBIT_ORBIT_HPP_
#define S2E_DYNAMICS_ORBIT_ORBIT_HPP_

#include <environment/global/celestial_information.hpp>
#include <environment/global/physical_constants.hpp>
#include <library/geodesy/geodetic_position.hpp>
#include <library/logger/loggable.hpp>
#include <library/math/constants.hpp>
#include <library/math/matrix.hpp>
#include <library/math/matrix_vector.hpp>
#include <library/math/quaternion.hpp>
#include <library/math/vector.hpp>

/**
 * @enum OrbitPropagateMode
 * @brief Propagation mode of orbit
 */
enum class OrbitPropagateMode {
  kRk4 = 0,        //!< 4th order Runge-Kutta propagation with disturbances and thruster maneuver
  kSgp4,           //!< SGP4 propagation using TLE without thruster maneuver
  kRelativeOrbit,  //!< Relative dynamics (for formation flying simulation)
  kKepler,         //!< Kepler orbit propagation without disturbances and thruster maneuver
  kEncke           //!< Encke orbit propagation with disturbances and thruster maneuver
};

/**
 * @enum OrbitInitializeMode
 * @brief Initialize mode of orbit
 */
enum class OrbitInitializeMode {
  kDefault = 0,                  //!< Default
  kInertialPositionAndVelocity,  //!< Position and velocity in the inertial frame
  kOrbitalElements,              //!< Orbital elements
};

/**
 * @class Orbit
 * @brief Base class of orbit propagation
 */
class Orbit : public ILoggable {
 public:
  /**
   * @fn Orbit
   * @brief Constructor
   * @param [in] celestial_information: Celestial information
   */
  Orbit(const CelestialInformation* celestial_information) : celestial_information_(celestial_information) {}
  /**
   * @fn ~Orbit
   * @brief Destructor
   */
  virtual ~Orbit() {}

  /**
   * @fn Propagate
   * @brief Pure virtual function for orbit propagation
   * @param [in] end_time_s: End time of simulation [sec]
   * @param [in] current_time_jd: Current Julian day [day]
   */
  virtual void Propagate(double end_time_s, double current_time_jd) = 0;

  /**
   * @fn UpdateAtt
   * @brief Update attitude information
   * @param [in] quaternion_i2b: End time of simulation [sec]
   */
  inline void UpdateAtt(libra::Quaternion quaternion_i2b) { spacecraft_velocity_b_m_s_ = quaternion_i2b.frame_conv(spacecraft_velocity_i_m_s_); }

  // Getters
  /**
   * @fn GetIsCalcEnabled
   * @brief Return calculate flag
   */
  inline bool GetIsCalcEnabled() const { return is_calc_enabled_; }
  /**
   * @fn GetPropagateMode
   * @brief Return propagate mode
   */
  inline OrbitPropagateMode GetPropagateMode() const { return propagate_mode_; }
  /**
   * @fn GetSatPosition_i
   * @brief Return spacecraft position in the inertial frame [m]
   */
  inline libra::Vector<3> GetSatPosition_i() const { return spacecraft_position_i_m_; }
  /**
   * @fn GetSatPosition_ecef
   * @brief Return spacecraft position in the ECEF frame [m]
   */
  inline libra::Vector<3> GetSatPosition_ecef() const { return spacecraft_position_ecef_m_; }
  /**
   * @fn GetSatVelocity_i
   * @brief Return spacecraft velocity in the inertial frame [m/s]
   */
  inline libra::Vector<3> GetSatVelocity_i() const { return spacecraft_velocity_i_m_s_; }
  /**
   * @fn GetSatVelocity_b
   * @brief Return spacecraft velocity in the body fixed frame [m/s]
   */
  inline libra::Vector<3> GetSatVelocity_b() const { return spacecraft_velocity_b_m_s_; }
  /**
   * @fn GetSatVelocity_ecef
   * @brief Return spacecraft velocity in the ECEF frame [m/s]
   */
  inline libra::Vector<3> GetSatVelocity_ecef() const { return spacecraft_velocity_ecef_m_s_; }
  /**
   * @fn GetGeodeticPosition
   * @brief Return spacecraft position in the geodetic frame [m]
   */
  inline GeodeticPosition GetGeodeticPosition() const { return spacecraft_geodetic_position_; }

  // TODO delete the following functions
  inline double GetLat_rad() const { return spacecraft_geodetic_position_.GetLat_rad(); }
  inline double GetLon_rad() const { return spacecraft_geodetic_position_.GetLon_rad(); }
  inline double GetAlt_m() const { return spacecraft_geodetic_position_.GetAlt_m(); }
  inline libra::Vector<3> GetLatLonAlt() const {
    libra::Vector<3> vec;
    vec(0) = spacecraft_geodetic_position_.GetLat_rad();
    vec(1) = spacecraft_geodetic_position_.GetLon_rad();
    vec(2) = spacecraft_geodetic_position_.GetAlt_m();
    return vec;
  }

  // Setters
  /**
   * @fn SetIsCalcEnabled
   * @brief Set calculate flag
   */
  inline void SetIsCalcEnabled(bool is_calc_enabled) { is_calc_enabled_ = is_calc_enabled; }
  /**
   * @fn SetAcceleration_i
   * @brief Set acceleration in the inertial frame [m/s2]
   */
  inline void SetAcceleration_i(libra::Vector<3> acceleration_i_m_s2) { spacecraft_acceleration_i_m_s2_ = acceleration_i_m_s2; }
  /**
   * @fn AddForce_i
   * @brief Add force
   * @param [in] force_i: Force in the inertial frame [N]
   * @param [in] spacecraft_mass_kg: Mass of spacecraft [kg]
   */
  inline void AddForce_i(libra::Vector<3> force_i, double spacecraft_mass_kg) {
    force_i /= spacecraft_mass_kg;
    spacecraft_acceleration_i_m_s2_ += force_i;
  }
  /**
   * @fn AddAcceleration_i_m_s2
   * @brief Add acceleration in the inertial frame [m/s2]
   */
  inline void AddAcceleration_i_m_s2(libra::Vector<3> acceleration_i_m_s2) { spacecraft_acceleration_i_m_s2_ += acceleration_i_m_s2; }
  /**
   * @fn AddForce_i
   * @brief Add force
   * @param [in] force_b_N: Force in the body fixed frame [N]
   * @param [in] quaternion_i2b: Quaternion from the inertial frame to the body fixed frame
   * @param [in] spacecraft_mass_kg: Mass of spacecraft [kg]
   */
  inline void AddForce_b_N(libra::Vector<3> force_b_N, libra::Quaternion quaternion_i2b, double spacecraft_mass_kg) {
    auto force_i = quaternion_i2b.frame_conv_inv(force_b_N);
    AddForce_i(force_i, spacecraft_mass_kg);
  }

  /**
   * @fn CalcQuaternionI2LVLH
   * @brief Calculate quaternion from the inertial frame to the LVLH frame
   */
  libra::Quaternion CalcQuaternionI2LVLH() const;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

 protected:
  const CelestialInformation* celestial_information_;  //!< Celestial information

  // Settings
  bool is_calc_enabled_ = false;       //!< Calculate flag
  OrbitPropagateMode propagate_mode_;  //!< Propagation mode

  libra::Vector<3> spacecraft_position_i_m_;       //!< Spacecraft position in the inertial frame [m]
  libra::Vector<3> spacecraft_position_ecef_m_;    //!< Spacecraft position in the ECEF frame [m]
  GeodeticPosition spacecraft_geodetic_position_;  //!< Spacecraft position in the Geodetic frame

  libra::Vector<3> spacecraft_velocity_i_m_s_;     //!< Spacecraft velocity in the inertial frame [m/s]
  libra::Vector<3> spacecraft_velocity_b_m_s_;     //!< Spacecraft velocity in the body frame [m/s]
  libra::Vector<3> spacecraft_velocity_ecef_m_s_;  //!< Spacecraft velocity in the ECEF frame [m/s]

  libra::Vector<3> spacecraft_acceleration_i_m_s2_;  //!< Spacecraft acceleration in the inertial frame [m/s2]
                                                     //!< NOTE: Clear to zero at the end of the Propagate function

  // Frame Conversion TODO: consider other planet
  /**
   * @fn TransformEciToEcef
   * @brief Transform states from the ECI frame to ECEF frame
   */
  void TransformEciToEcef(void);
  /**
   * @fn TransformEcefToGeodetic
   * @brief Transform states from the ECEF frame to the geodetic frame
   */
  void TransformEcefToGeodetic(void);
};

OrbitInitializeMode SetOrbitInitializeMode(const std::string initialize_mode);

#endif  // S2E_DYNAMICS_ORBIT_ORBIT_HPP_
