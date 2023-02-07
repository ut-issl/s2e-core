/**
 * @file GNSSReceiver.h
 * @brief Class to emulate GNSS receiver
 */

#pragma once

#include <Dynamics/Dynamics.h>
#include <environment/global/gnss_satellites.hpp>
#include <environment/global/SimTime.h>
#include <Interface/LogOutput/ILoggable.h>

#include <Library/math/NormalRand.hpp>
#include <Library/math/Quaternion.hpp>

#include "../Abstract/ComponentBase.h"

using libra::Vector;

/**
 * @enum AntennaModel
 * @brief Antenna pattern model to emulate GNSS antenna
 */
enum AntennaModel {
  SIMPLE,  //!< Simple model which can get navigation data when the antenna points anti-earth direction
  CONE,    //!< Cone antenna pattern
};

/**
 * @struct GnssInfo
 * @brief Information of GNSS satellites
 */
typedef struct _gnssinfo {
  std::string ID;    //!< ID of GNSS satellites
  double latitude;   //!< Latitude on the antenna frame [rad]
  double longitude;  //!< Longitude on the antenna frame [rad]
  double distance;   //!< Distance between the GNSS satellite and the GNSS receiver antenna [m]
} GnssInfo;

/**
 * @class GNSSReceiver
 * @brief Class to emulate GNSS receiver
 */
class GNSSReceiver : public ComponentBase, public ILoggable {
 public:
  /**
   * @fn GNSSReceiver
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] gnss_id: GNSS satellite number defined by GNSS system
   * @param [in] ch_max: Maximum number of channels
   * @param [in] antenna_model: Antenna model
   * @param [in] ant_pos_b: GNSS antenna position at the body-fixed frame [m]
   * @param [in] q_b2c: Quaternion from body frame to component frame (antenna frame)
   * @param [in] half_width: Half width of the antenna cone model [rad]
   * @param [in] noise_std: Standard deviation of normal random noise in the ECI frame [m]
   * @param [in] dynamics: Dynamics information
   * @param [in] gnss_satellites: GNSS Satellites information
   * @param [in] simtime: Simulation time information
   */
  GNSSReceiver(const int prescaler, ClockGenerator* clock_gen, const int id, const std::string gnss_id, const int ch_max,
               const AntennaModel antenna_model, const Vector<3> ant_pos_b, const Quaternion q_b2c, const double half_width,
               const Vector<3> noise_std, const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimTime* simtime);
  /**
   * @fn GNSSReceiver
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] power_port: Power port
   * @param [in] gnss_id: GNSS satellite number defined by GNSS system
   * @param [in] ch_max: Maximum number of channels
   * @param [in] antenna_model: Antenna model
   * @param [in] ant_pos_b: GNSS antenna position at the body-fixed frame [m]
   * @param [in] q_b2c: Quaternion from body frame to component frame (antenna frame)
   * @param [in] half_width: Half width of the antenna cone model [rad]
   * @param [in] noise_std: Standard deviation of normal random noise in the ECI frame [m]
   * @param [in] dynamics: Dynamics information
   * @param [in] gnss_satellites: GNSS Satellites information
   * @param [in] simtime: Simulation time information
   */
  GNSSReceiver(const int prescaler, ClockGenerator* clock_gen, PowerPort* power_port, const int id, std::string gnss_id, const int ch_max,
               const AntennaModel antenna_model, const Vector<3> ant_pos_b, const Quaternion q_b2c, const double half_width,
               const Vector<3> noise_std, const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimTime* simtime);

  // Override functions for ComponentBase
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(int count);

  // Getter
  /**
   * @fn GetGnssInfo
   * @brief Return GNSS satellite information
   * @param [in] ch: Channel number
   */
  inline const GnssInfo GetGnssInfo(int ch) const { return vec_gnssinfo_[ch]; };
  /**
   * @fn GetPositionECI
   * @brief Return Observed position in the ECI frame [m]
   */
  inline const Vector<3> GetPositionECI(void) const { return position_eci_; }
  /**
   * @fn GetPositionECEF
   * @brief Return Observed position in the ECEF frame [m]
   */
  inline const Vector<3> GetPositionECEF(void) const { return position_ecef_; }
  /**
   * @fn GetPositionLLH
   * @brief Return Observed position in the LLH frame [m]
   */
  inline const Vector<3> GetPositionLLH(void) const { return position_llh_; }
  /**
   * @fn GetVelocityECI
   * @brief Return Observed velocity in the ECI frame [m/s]
   */
  inline const Vector<3> GetVelocityECI(void) const { return velocity_eci_; }
  /**
   * @fn GetVelocityECEF
   * @brief Return Observed velocity in the ECEF frame [m/s]
   */
  inline const Vector<3> GetVelocityECEF(void) const { return velocity_ecef_; }

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
  // Parameters for receiver
  const int id_;                  //!< Receiver ID
  const int ch_max_;              //!< Maximum number of channels
  Vector<3> antenna_position_b_;  //!< GNSS antenna position at the body-fixed frame [m]
  Quaternion q_b2c_;              //!< Quaternion from body frame to component frame (antenna frame)

  libra::NormalRand nrs_eci_x_, nrs_eci_y_, nrs_eci_z_;  //!< Random noise for each axis

  double half_width_ = 0.0;     //!< Half width of the antenna cone model [rad]
  std::string gnss_id_;         //!< GNSS satellite number defined by GNSS system
  AntennaModel antenna_model_;  //!< Antenna model

  // Calculated values
  Vector<3> position_eci_{0.0};         //!< Observed position in the ECI frame [m]
  Vector<3> velocity_eci_{0.0};         //!< Observed velocity in the ECI frame [m/s]
  Vector<3> position_ecef_{0.0};        //!< Observed position in the ECEF frame [m]
  Vector<3> velocity_ecef_{0.0};        //!< Observed velocity in the ECEF frame [m/s]
  Vector<3> position_llh_{0.0};         //!< Observed position in the geodetic frame [rad,rad,m] TODO: use GeodeticPosition class
  UTC utc_ = {2000, 1, 1, 0, 0, 0.0};   //!< Observed time in UTC [year, month, day, hour, min, sec]
  unsigned int gpstime_week_ = 0;       //!< Observed GPS time week part
  double gpstime_sec_ = 0.0;            //!< Observed GPS time second part
  int is_gnss_sats_visible_ = 0;        //!< Flag for GNSS satellite is visible or not
  int gnss_sats_visible_num_ = 0;       //!< Number of visible GNSS satellites
  std::vector<GnssInfo> vec_gnssinfo_;  //!< Information List of visible GNSS satellites

  // References
  const Dynamics* dynamics_;               //!< Dynamics of spacecraft
  const GnssSatellites* gnss_satellites_;  //!< Information of GNSS satellites
  const SimTime* simtime_;                 //!< Simulation time

  // Internal Functions
  /**
   * @fn CheckAntenna
   * @brief Check the antenna can detect GNSS signal
   * @note This function just calls other check functions according to the antenna mode
   * @param [in] location_true: True position of the spacecraft in the ECI frame [m]
   * @param [in] q_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   */
  void CheckAntenna(Vector<3> location_true, Quaternion q_i2b);
  /**
   * @fn CheckAntennaSimple
   * @brief Check the antenna can detect GNSS signal with Simple mode
   * @note GNSS satellites are visible when antenna directs anti-earth direction
   * @param [in] location_true: True position of the spacecraft in the ECI frame [m]
   * @param [in] q_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   */
  void CheckAntennaSimple(Vector<3> location_true, Quaternion q_i2b);
  /**
   * @fn CheckAntennaCone
   * @brief Check the antenna can detect GNSS signal with Cone mode
   * @note The visible GNSS satellites are counted by using GNSS satellite position and the antenna direction with cone antenna pattern
   * @param [in] location_true: True position of the spacecraft in the ECI frame [m]
   * @param [in] q_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   */
  void CheckAntennaCone(Vector<3> location_true, Quaternion q_i2b);
  /**
   * @fn SetGnssInfo
   * @brief Calculate and set the GnssInfo values of target GNSS satellite
   * @param [in] ant2gnss_i: Position vector from the antenna to the GNSS satellites in the ECI frame
   * @param [in] q_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   * @param [in] gnss_id: ID of target GNSS satellite
   */
  void SetGnssInfo(Vector<3> ant2gnss_i, Quaternion q_i2b, std::string gnss_id);
  /**
   * @fn AddNoise
   * @brief Substitutional method for "Measure" in other sensor models inherited SensorBase class
   * @param [in] location_true_eci: True position of the spacecraft in the ECI frame [m]
   * @param [in] location_true_ecef: True position of the spacecraft in the ECEF frame [m]
   */
  void AddNoise(Vector<3> location_true_eci, Vector<3> location_true_ecef);
  /**
   * @fn ConvertJulianDayToGPSTime
   * @brief Convert Julian day to GPS time
   * @param [in] JulianDay: Julian day
   */
  void ConvertJulianDayToGPSTime(const double JulianDay);
};
