/**
 * @file gnss_receiver.hpp
 * @brief Class to emulate GNSS receiver
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_GNSS_RECEIVER_HPP_
#define S2E_COMPONENTS_REAL_AOCS_GNSS_RECEIVER_HPP_

#include <dynamics/dynamics.hpp>
#include <environment/global/gnss_satellites.hpp>
#include <environment/global/simulation_time.hpp>
#include <logger/loggable.hpp>
#include <math_physics/geodesy/geodetic_position.hpp>
#include <math_physics/math/quaternion.hpp>
#include <math_physics/randomization/normal_randomization.hpp>

#include "../../base/component.hpp"

namespace s2e::components {

/**
 * @enum AntennaModel
 * @brief Antenna pattern model to emulate GNSS antenna
 */
enum class AntennaModel {
  kSimple,  //!< Simple model which can get navigation data when the antenna points anti-earth direction
  kCone,    //!< Cone antenna pattern
};

/**
 * @struct GnssInfo
 * @brief Information of GNSS satellites
 */
typedef struct _gnss_info {
  size_t gnss_id;        //!< ID of GNSS satellites
  double latitude_rad;   //!< Latitude on the antenna frame [rad]
  double longitude_rad;  //!< Longitude on the antenna frame [rad]
  double distance_m;     //!< Distance between the GNSS satellite and the GNSS receiver antenna [m]
} GnssInfo;

/**
 * @class GnssReceiver
 * @brief Class to emulate GNSS receiver
 */
class GnssReceiver : public Component, public logger::ILoggable {
 public:
  /**
   * @fn GnssReceiver
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: Component ID
   * @param [in] antenna_model: Antenna model
   * @param [in] antenna_position_b_m: GNSS antenna position at the body-fixed frame [m]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame (antenna frame)
   * @param [in] half_width_deg: Half width of the antenna cone model [deg]
   * @param [in] klobuchar_alpha: Klobuchar alpha coefficients
   * @param [in] klobuchar_beta: Klobuchar beta coefficients
   * @param [in] pseudorange_noise_standard_deviation_m: Standard deviation of normal random noise for pseudorange [m]
   * @param [in] position_noise_standard_deviation_ecef_m: Standard deviation of normal random noise for position in the ECEF frame [m]
   * @param [in] velocity_noise_standard_deviation_ecef_m_s: Standard deviation of normal random noise for velocity in the ECEF frame [m/s]
   * @param [in] is_log_pseudorange_enabled: Enable flag to log output pseudorange
   * @param [in] dynamics: Dynamics information
   * @param [in] gnss_satellites: GNSS Satellites information
   * @param [in] simulation_time: Simulation time information
   */
  GnssReceiver(const int prescaler, environment::ClockGenerator* clock_generator, const size_t component_id, const AntennaModel antenna_model,
               const math::Vector<3> antenna_position_b_m, const math::Quaternion quaternion_b2c, const double half_width_deg, const math::Vector<4> klobuchar_alpha,
               const math::Vector<4> klobuchar_beta,
               const double pseudorange_noise_standard_deviation_m, const math::Vector<3> position_noise_standard_deviation_ecef_m,
               const math::Vector<3> velocity_noise_standard_deviation_ecef_m_s, const bool is_log_pseudorange_enabled,
               const dynamics::Dynamics* dynamics, const environment::GnssSatellites* gnss_satellites,
               const environment::SimulationTime* simulation_time);
  /**
   * @fn GnssReceiver
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] antenna_model: Antenna model
   * @param [in] antenna_position_b_m: GNSS antenna position at the body-fixed frame [m]
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame (antenna frame)
   * @param [in] half_width_deg: Half width of the antenna cone model [rad]
   * @param [in] klobuchar_alpha: Klobuchar alpha coefficients
   * @param [in] klobuchar_beta: Klobuchar beta coefficients
   * @param [in] pseudorange_noise_standard_deviation_m: Standard deviation of normal random noise for pseudorange [m]
   * @param [in] position_noise_standard_deviation_ecef_m: Standard deviation of normal random noise for position in the ECEF frame [m]
   * @param [in] velocity_noise_standard_deviation_ecef_m_s: Standard deviation of normal random noise for velocity in the ECEF frame [m/s]
   * @param [in] is_log_pseudorange_enabled: Enable flag to log output pseudorange
   * @param [in] dynamics: Dynamics information
   * @param [in] gnss_satellites: GNSS Satellites information
   * @param [in] simulation_time: Simulation time information
   */
  GnssReceiver(const int prescaler, environment::ClockGenerator* clock_generator, PowerPort* power_port, const size_t component_id,
               const AntennaModel antenna_model, const math::Vector<3> antenna_position_b_m, const math::Quaternion quaternion_b2c,
               const double half_width_deg, const math::Vector<4> klobuchar_alpha,
               const math::Vector<4> klobuchar_beta, const double pseudorange_noise_standard_deviation_m,
               const math::Vector<3> position_noise_standard_deviation_ecef_m, const math::Vector<3> velocity_noise_standard_deviation_ecef_m_s,
               const bool is_log_pseudorange_enabled, const dynamics::Dynamics* dynamics, const environment::GnssSatellites* gnss_satellites,
               const environment::SimulationTime* simulation_time);

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
  inline const GnssInfo GetGnssInfo(const size_t channel) const { return gnss_information_list_[channel]; };
  /**
   * @fn GetMeasuredPosition_ecef_m
   * @brief Return Observed position in the ECEF frame [m]
   */
  inline const math::Vector<3> GetMeasuredPosition_ecef_m(void) const { return position_ecef_m_; }
  /**
   * @fn GetMeasuredGeodeticPosition
   * @brief Return Observed position in the LLH frame [m]
   */
  inline const geodesy::GeodeticPosition GetMeasuredGeodeticPosition(void) const { return geodetic_position_; }
  /**
   * @fn GetMeasuredVelocity_ecef_m_s
   * @brief Return Observed velocity in the ECEF frame [m/s]
   */
  inline const math::Vector<3> GetMeasuredVelocity_ecef_m_s(void) const { return velocity_ecef_m_s_; }

  // Override logger::ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of logger::ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of logger::ILoggable
   */
  virtual std::string GetLogValue() const;

 protected:
  // GNSS satellite number definition
  // TODO: Move to initialized file?
  static const size_t kNumberOfGpsSatellite = 32;      //!< Number of GPS satellites
  static const size_t kNumberOfGlonassSatellite = 26;  //!< Number of GLONASS satellites
  static const size_t kNumberOfGalileoSatellite = 28;  //!< Number of Galileo satellites
  static const size_t kNumberOfBeidouSatellite = 62;   //!< Number of BeiDou satellites
  static const size_t kNumberOfQzssSatellite = 5;      //!< Number of QZSS satellites
  static const size_t kNumberOfNavicSatellite = 7;     //!< Number of NavIC satellites

  static const size_t kTotalNumberOfGnssSatellite = kNumberOfGpsSatellite + kNumberOfGlonassSatellite + kNumberOfGalileoSatellite +
                                                    kNumberOfBeidouSatellite + kNumberOfQzssSatellite +
                                                    kNumberOfNavicSatellite;  //<! Total number of GNSS satellites

  // Parameters for receiver
  const size_t component_id_;  //!< Receiver ID

  // Antenna
  math::Vector<3> antenna_position_b_m_;  //!< GNSS antenna position at the body-fixed frame [m]
  math::Quaternion quaternion_b2c_;       //!< Quaternion from body frame to component frame (antenna frame)
  double half_width_deg_ = 0.0;           //!< Half width of the antenna cone model [deg]
  AntennaModel antenna_model_;            //!< Antenna model

  // GNSS observation
  math::Vector<4> klobuchar_alpha_;
  math::Vector<4> klobuchar_beta_;
  randomization::NormalRand pseudorange_random_noise_m_;                      //!< Random noise for pseudorange [m]
  std::vector<double> pseudorange_list_m_{kTotalNumberOfGnssSatellite, 0.0};  //!< Pseudorange list for each GPS satellite
  bool is_logged_pseudorange_;                                                //!< Flag for log output of pseudorange

  // Simple position observation
  randomization::NormalRand position_random_noise_ecef_m_[3];    //!< Random noise for position at the ECEF frame [m]
  randomization::NormalRand velocity_random_noise_ecef_m_s_[3];  //!< Random noise for velocity at the ECEF frame [m]
  math::Vector<3> position_ecef_m_{0.0};                         //!< Observed position in the ECEF frame [m]
  math::Vector<3> velocity_ecef_m_s_{0.0};                       //!< Observed velocity in the ECEF frame [m/s]
  geodesy::GeodeticPosition geodetic_position_;                  //!< Observed position in the geodetic frame

  // Time observation
  environment::UTC utc_ = {2000, 1, 1, 0, 0, 0.0};  //!< Observed time in UTC [year, month, day, hour, min, sec]
  unsigned int gps_time_week_ = 0;                  //!< Observed GPS time week part
  double gps_time_s_ = 0.0;                         //!< Observed GPS time second part

  // Satellite visibility
  bool is_gnss_visible_ = false;                 //!< Flag for GNSS satellite is visible or not
  size_t visible_satellite_number_ = 0;          //!< Number of visible GNSS satellites
  std::vector<GnssInfo> gnss_information_list_;  //!< Information List of visible GNSS satellites

  // References
  const dynamics::Dynamics* dynamics_;                  //!< Dynamics of spacecraft
  const environment::GnssSatellites* gnss_satellites_;  //!< Information of GNSS satellites
  const environment::SimulationTime* simulation_time_;  //!< Simulation time

  // Internal Functions
  /**
   * @fn CheckAntenna
   * @brief Check the antenna can detect GNSS signal
   * @note This function just calls other check functions according to the antenna mode
   * @param [in] position_true_i_m: True position of the spacecraft in the ECI frame [m]
   * @param [in] quaternion_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   */
  void CheckAntenna(const math::Vector<3> position_true_i_m, const math::Quaternion quaternion_i2b);
  /**
   * @fn CheckAntennaSimple
   * @brief Check the antenna can detect GNSS signal with Simple mode
   * @note GNSS satellites are visible when antenna directs anti-earth direction
   * @param [in] position_true_i_m: True position of the spacecraft in the ECI frame [m]
   * @param [in] quaternion_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   */
  void CheckAntennaSimple(const math::Vector<3> position_true_i_m, const math::Quaternion quaternion_i2b);
  /**
   * @fn CheckAntennaCone
   * @brief Check the antenna can detect GNSS signal with Cone mode
   * @note The visible GNSS satellites are counted by using GNSS satellite position and the antenna direction with cone antenna pattern
   * @param [in] position_true_i_m: True position of the spacecraft in the ECI frame [m]
   * @param [in] quaternion_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   */
  void CheckAntennaCone(const math::Vector<3> position_true_i_m, const math::Quaternion quaternion_i2b);
  /**
   * @fn SetGnssInfo
   * @brief Calculate and set the GnssInfo values of target GNSS satellite
   * @param [in] antenna_to_satellite_i_m: Position vector from the antenna to the GNSS satellites in the ECI frame
   * @param [in] quaternion_i2b: True attitude of the spacecraft expressed by quaternion from the inertial frame to the body-fixed frame
   * @param [in] gnss_system_id: ID of target GNSS satellite
   */
  void SetGnssInfo(const math::Vector<3> antenna_to_satellite_i_m, const math::Quaternion quaternion_i2b, const size_t gnss_system_id);
  /**
   * @fn CalcGeometricDistance
   * @brief Calculate the geometric distance between the GNSS satellite and the GNSS receiver antenna
   * @param [in] gnss_system_id: ID of target GNSS satellite
   * @return Geometric distance between the GNSS satellite and the GNSS receiver antenna [m]
   */
  double CalcGeometricDistance_m(const size_t gnss_system_id);
  /**
   * @fn CalcElevationAzimuth_rad
   * @brief Calculate the elevation and azimuth angles of the GNSS satellite from the receiver antenna
   * @param [in] gnss_system_id: ID of target GNSS satellite
   * @return Vector of elevation and azimuth angles in radians [rad]
   */
  std::vector<double> CalcElevationAzimuth_rad(const size_t gnss_system_id);
  /**
   * @fn CalcIonosphericDelay
   * @brief Calculate the slant ionospheric delay using the Klobuchar model
   * @param [in] gnss_system_id: ID of target GNSS satellite
   * @param [in] band_id: ID of target GNSS satellite frequency band
   * @return Ionospheric delay [m]
   */
  double CalcIonosphericDelay_m(const size_t gnss_system_id, const size_t band_id);
  /**
   * @fn CalcPseudorange
   * @brief Calculate the pseudorange between the GNSS satellite and the GNSS receiver antenna
   * @param [in] gnss_system_id: ID of target GNSS satellite
   * @return Pseudorange between the GNSS satellite and the GNSS receiver antenna [m]
   */
  double CalcPseudorange_m(const size_t gnss_id);
  /**
   * @fn SetGnssObservationList
   * @brief Calculate and set the GNSS observation list for each GNSS satellite
   */
  void SetGnssObservationList();
  /**
   * @fn AddNoise
   * @brief Substitutional method for "Measure" in other sensor models inherited Sensor class
   * @param [in] position_true_ecef_m: True position of the spacecraft in the ECEF frame [m]
   * @param [in] velocity_true_ecef_m_s: True velocity of the spacecraft in the ECEF frame [m/s]
   */
  void AddNoise(const math::Vector<3> position_true_ecef_m, const math::Vector<3> velocity_true_ecef_m_s);
  /**
   * @fn ConvertJulianDayToGpsTime
   * @brief Convert Julian day to GPS time
   * @param [in] julian_day: Julian day
   */
  void ConvertJulianDayToGpsTime(const double julian_day);
};

/**
 * @fn SetAntennaModel
 * @brief Set AntennaModel by string
 * @param [in] antenna_model: Antenna model name
 * @return antenna model
 */
AntennaModel SetAntennaModel(const std::string antenna_model);

/**
 * @fn InitGnssReceiver
 * @brief Initialize functions for GNSS Receiver without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] component_id: Sensor ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 * @param [in] gnss_satellites: GNSS satellites information
 * @param [in] simulation_time: Simulation time information
 */
GnssReceiver InitGnssReceiver(environment::ClockGenerator* clock_generator, const size_t component_id, const std::string file_name,
                              const dynamics::Dynamics* dynamics, const environment::GnssSatellites* gnss_satellites,
                              const environment::SimulationTime* simulation_time);
/**
 * @fn InitGnssReceiver
 * @brief Initialize functions for GNSS Receiver with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] component_id: Sensor ID
 * @param [in] power_port: Power port
 * @param [in] file_name: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 * @param [in] gnss_satellites: GNSS satellites information
 * @param [in] simulation_time: Simulation time information
 */
GnssReceiver InitGnssReceiver(environment::ClockGenerator* clock_generator, PowerPort* power_port, const size_t component_id,
                              const std::string file_name, const dynamics::Dynamics* dynamics, const environment::GnssSatellites* gnss_satellites,
                              const environment::SimulationTime* simulation_time);

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_REAL_AOCS_GNSS_RECEIVER_HPP_
