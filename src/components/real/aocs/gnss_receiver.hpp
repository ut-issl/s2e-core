/**
 * @file gnss_receiver.hpp
 * @brief Class to emulate GNSS receiver
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_GNSS_RECEIVER_HPP_
#define S2E_COMPONENTS_REAL_AOCS_GNSS_RECEIVER_HPP_

#include <dynamics/dynamics.hpp>
#include <environment/global/gnss_satellites.hpp>
#include <environment/global/simulation_time.hpp>
#include <library/logger/loggable.hpp>
#include <library/math/quaternion.hpp>
#include <library/randomization/normal_randomization.hpp>

#include "../../base/component.hpp"

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
typedef struct _gnss_info {
  std::string ID;        //!< ID of GNSS satellites
  double latitude_rad;   //!< Latitude on the antenna frame [rad]
  double longitude_rad;  //!< Longitude on the antenna frame [rad]
  double distance_m;     //!< Distance between the GNSS satellite and the GNSS receiver antenna [m]
} GnssInfo;

/**
 * @class GnssReceiver
 * @brief Class to emulate GNSS receiver
 */
class GnssReceiver : public Component, public ILoggable {
 public:
  /**
   * @fn GnssReceiver
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: Component ID
   * @param [in] gnss_id: GNSS satellite number defined by GNSS system
   * @param [in] max_channel: Maximum number of channels
   * @param [in] antenna_model: Antenna model
   * @param [in] antenna_position_b_m: GNSS antenna position at the body-fixed frame [m]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame (antenna frame)
   * @param [in] half_width_rad: Half width of the antenna cone model [rad]
   * @param [in] noise_standard_deviation_m: Standard deviation of normal random noise in the ECI frame [m]
   * @param [in] dynamics: Dynamics information
   * @param [in] gnss_satellites: GNSS Satellites information
   * @param [in] simulation_time: Simulation time information
   */
  GnssReceiver(const int prescaler, ClockGenerator* clock_generator, const int component_id, const std::string gnss_id, const int max_channel,
               const AntennaModel antenna_model, const libra::Vector<3> antenna_position_b_m, const libra::Quaternion quaternion_b2c,
               const double half_width_rad, const libra::Vector<3> noise_standard_deviation_m, const Dynamics* dynamics,
               const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time);
  /**
   * @fn GnssReceiver
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] gnss_id: GNSS satellite number defined by GNSS system
   * @param [in] max_channel: Maximum number of channels
   * @param [in] antenna_model: Antenna model
   * @param [in] antenna_position_b_m: GNSS antenna position at the body-fixed frame [m]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame (antenna frame)
   * @param [in] half_width_rad: Half width of the antenna cone model [rad]
   * @param [in] noise_standard_deviation_m: Standard deviation of normal random noise in the ECI frame [m]
   * @param [in] dynamics: Dynamics information
   * @param [in] gnss_satellites: GNSS Satellites information
   * @param [in] simulation_time: Simulation time information
   */
  GnssReceiver(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id, std::string gnss_id,
               const int max_channel, const AntennaModel antenna_model, const libra::Vector<3> antenna_position_b_m,
               const libra::Quaternion quaternion_b2c, const double half_width_rad, const libra::Vector<3> noise_standard_deviation_m,
               const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimulationTime* simulation_time);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count);

  // Getter
  /**
   * @fn GetGnssInfo
   * @brief Return GNSS satellite information
   * @param [in] channel: Channel number
   */
  inline const GnssInfo GetGnssInfo(int channel) const { return gnss_information_list_[channel]; };
  /**
   * @fn GetMeasuredPosition_i_m
   * @brief Return Observed position in the ECI frame [m]
   */
  inline const libra::Vector<3> GetMeasuredPosition_i_m(void) const { return position_eci_m_; }
  /**
   * @fn GetMeasuredPosition_ecef_m
   * @brief Return Observed position in the ECEF frame [m]
   */
  inline const libra::Vector<3> GetMeasuredPosition_ecef_m(void) const { return position_ecef_m_; }
  /**
   * @fn GetMeasuredGeodeticPosition
   * @brief Return Observed position in the LLH frame [m]
   */
  inline const libra::Vector<3> GetMeasuredGeodeticPosition(void) const { return position_llh_; }
  /**
   * @fn GetMeasuredVelocity_i_m_s
   * @brief Return Observed velocity in the ECI frame [m/s]
   */
  inline const libra::Vector<3> GetMeasuredVelocity_i_m_s(void) const { return velocity_eci_m_s_; }
  /**
   * @fn GetMeasuredVelocity_ecef_m_s
   * @brief Return Observed velocity in the ECEF frame [m/s]
   */
  inline const libra::Vector<3> GetMeasuredVelocity_ecef_m_s(void) const { return velocity_ecef_m_s_; }

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
  const int component_id_;                 //!< Receiver ID
  const int max_channel_;                  //!< Maximum number of channels
  libra::Vector<3> antenna_position_b_m_;  //!< GNSS antenna position at the body-fixed frame [m]
  libra::Quaternion quaternion_b2c_;       //!< Quaternion from body frame to component frame (antenna frame)

  libra::NormalRand random_noise_i_x_, random_noise_i_y_, random_noise_i_z_;  //!< Random noise for each axis

  double half_width_rad_ = 0.0;  //!< Half width of the antenna cone model [rad]
  std::string gnss_id_;          //!< GNSS satellite number defined by GNSS system
  AntennaModel antenna_model_;   //!< Antenna model

  // Calculated values
  libra::Vector<3> position_eci_m_{0.0};         //!< Observed position in the ECI frame [m]
  libra::Vector<3> velocity_eci_m_s_{0.0};       //!< Observed velocity in the ECI frame [m/s]
  libra::Vector<3> position_ecef_m_{0.0};        //!< Observed position in the ECEF frame [m]
  libra::Vector<3> velocity_ecef_m_s_{0.0};      //!< Observed velocity in the ECEF frame [m/s]
  libra::Vector<3> position_llh_{0.0};           //!< Observed position in the geodetic frame [rad,rad,m] TODO: use GeodeticPosition class
  UTC utc_ = {2000, 1, 1, 0, 0, 0.0};            //!< Observed time in UTC [year, month, day, hour, min, sec]
  unsigned int gps_time_week_ = 0;               //!< Observed GPS time week part
  double gps_time_s_ = 0.0;                      //!< Observed GPS time second part
  int is_gnss_visible_ = 0;                      //!< Flag for GNSS satellite is visible or not
  int visible_satellite_number_ = 0;             //!< Number of visible GNSS satellites
  std::vector<GnssInfo> gnss_information_list_;  //!< Information List of visible GNSS satellites

  // References
  const Dynamics* dynamics_;               //!< Dynamics of spacecraft
  const GnssSatellites* gnss_satellites_;  //!< Information of GNSS satellites
  const SimulationTime* simulation_time_;  //!< Simulation time

  // Internal Functions
  /**
   * @fn CheckAntenna
   * @brief Check the antenna can detect GNSS signal
   * @note This function just calls other check functions according to the antenna mode
   * @param [in] position_true_i_m: True position of the spacecraft in the ECI frame [m]
   * @param [in] quaternion_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   */
  void CheckAntenna(libra::Vector<3> position_true_i_m, libra::Quaternion quaternion_i2b);
  /**
   * @fn CheckAntennaSimple
   * @brief Check the antenna can detect GNSS signal with Simple mode
   * @note GNSS satellites are visible when antenna directs anti-earth direction
   * @param [in] position_true_i_m: True position of the spacecraft in the ECI frame [m]
   * @param [in] quaternion_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   */
  void CheckAntennaSimple(libra::Vector<3> position_true_i_m, libra::Quaternion quaternion_i2b);
  /**
   * @fn CheckAntennaCone
   * @brief Check the antenna can detect GNSS signal with Cone mode
   * @note The visible GNSS satellites are counted by using GNSS satellite position and the antenna direction with cone antenna pattern
   * @param [in] position_true_i_m: True position of the spacecraft in the ECI frame [m]
   * @param [in] quaternion_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   */
  void CheckAntennaCone(libra::Vector<3> position_true_i_m, libra::Quaternion quaternion_i2b);
  /**
   * @fn SetGnssInfo
   * @brief Calculate and set the GnssInfo values of target GNSS satellite
   * @param [in] antenna_to_satellite_i_m: Position vector from the antenna to the GNSS satellites in the ECI frame
   * @param [in] quaternion_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   * @param [in] gnss_id: ID of target GNSS satellite
   */
  void SetGnssInfo(libra::Vector<3> antenna_to_satellite_i_m, libra::Quaternion quaternion_i2b, std::string gnss_id);
  /**
   * @fn AddNoise
   * @brief Substitutional method for "Measure" in other sensor models inherited Sensor class
   * @param [in] position_true_i_m: True position of the spacecraft in the ECI frame [m]
   * @param [in] position_true_ecef_m: True position of the spacecraft in the ECEF frame [m]
   */
  void AddNoise(libra::Vector<3> position_true_i_m, libra::Vector<3> position_true_ecef_m);
  /**
   * @fn ConvertJulianDayToGPSTime
   * @brief Convert Julian day to GPS time
   * @param [in] julian_day: Julian day
   */
  void ConvertJulianDayToGPSTime(const double julian_day);
};

#endif  // S2E_COMPONENTS_REAL_AOCS_GNSS_RECEIVER_HPP_
