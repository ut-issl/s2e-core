/**
 * @file sp3_file_reader.hpp
 * @brief A class to read the SP3 (Extended Standard Product 3) format file and provide functions to access the data
 * @note Support version: SP3-d
 *       Ref: https://files.igs.org/pub/data/format/sp3d.pdf?_ga=2.115202830.823990648.1664976786-1792483726.1664976785
 */

#ifndef S2E_LIBRARY_GNSS_SP3_FILE_READER_HPP_
#define S2E_LIBRARY_GNSS_SP3_FILE_READER_HPP_

#include <stdint.h>

#include <map>
#include <math_physics/math/vector.hpp>
#include <math_physics/time_system/date_time_format.hpp>
#include <math_physics/time_system/gps_time.hpp>
#include <string>
#include <vector>

#define SP3_BAD_CLOCK_VALUE (999999.999999)
#define SP3_BAD_POSITION_VALUE (0.000000)

/**
 * @enum Sp3Mode
 * @brief Data mode of SP3 file to define including data
 */
enum class Sp3Mode {
  kPosition,  //!< Position and clock data mode
  kVelocity,  //!< Velocity and clock rate data mode
  kOther,     //!< Undefined mode
};

/**
 * @enum Sp3OrbitType
 * @brief Orbit type defined in SP3
 */
enum class Sp3OrbitType {
  kFitted,        //!< Fitted
  kExtrapolated,  //!< Extrapolated or predicted
  kBroadcast,     //!< Broadcast
  kHelmert,       //!< Fitted after Helmert transformation
  kOther,         //!< Undefined mode
};

/**
 * @struct Sp3Header
 * @brief SP3 file header information
 */
struct Sp3Header {
  // 1st line information
  // version -> not implemented yet
  Sp3Mode mode_;                   //!< position or velocity
  DateTime start_epoch_;           //!< Time of start epoch
  size_t number_of_epoch_ = 0;     //!< Number of epoch in the SP3 file
  std::string used_data_;          //!< Used data to generate the SP3 file
  std::string coordinate_system_;  //!< Coordinate system for the position and velocity data
  Sp3OrbitType orbit_type_;        //!< Orbit type
  std::string agency_name_;        //!< Agency name who generates the SP3 file

  // 2nd line information
  GpsTime start_gps_time_;                        //!< Start time of orbit
  double epoch_interval_s_ = 1.0;                 //!< Epoch interval (0.0, 100000.0)
  size_t start_time_mjday_;                       //!< Start time of the orbit data (44244 = 6th Jan. 1980) [Modified Julian day]
  double start_time_mjday_fractional_day_ = 0.0;  //!< Fractional part of the start time [0.0, 1.0) [day]

  // 3rd line information
  size_t number_of_satellites_ = 0;  //!< Number of satellites in the SP3 file
  // Used satellite IDs (3rd to 11the line)
  std::vector<std::string> satellite_ids_;  //!< Satellite ID list

  // Accuracy of each satellite (12th to 20th line)
  std::vector<uint16_t> satellite_accuracy_;  //!< List of accuracy of each satellite

  // 21st line information
  std::string file_type_;    //!< File type
  std::string time_system_;  //!< Time system for the position and velocity data

  // 22nd line information
  // Not important

  // 23rd line information
  double base_number_position_ = 1.0;  //!< Floating point base number used for computing the standard deviations of position and velocity
  double base_number_clock_ = 1.0;     //!< Floating point base number used for computing the standard deviations of clock and clock-rate correction

  // 24th - 26th line information
  // Not important
};

/**
 * @struct Sp3PositionClock
 * @brief SP3 file position and clock information
 * @note The coordinate system of the position is defined in the SP3 header
 */
struct Sp3PositionClock {
  std::string satellite_id_;                              //!< GNSS satellite ID
  math::Vector<3> position_km_{SP3_BAD_POSITION_VALUE};  //!< Satellite position [km]
  double clock_us_ = SP3_BAD_CLOCK_VALUE;                 //!< Satellite clock offset [us]
  math::Vector<3> position_standard_deviation_{0.0};     //!< Satellite position standard deviation [-]
  double clock_standard_deviation_ = 0.0;                 //!< Satellite clock offset standard deviation [-]
  bool clock_event_flag_ = false;                         //!< true when clock discontinuity is happened
  bool clock_prediction_flag_ = false;                    //!< true when clock data is predicted
  bool maneuver_flag_ = false;                            //!< true when orbit maneuver is happened in last 50 minutes
  bool orbit_prediction_flag_ = false;                    //!< true when orbit data is predicted
};

/**
 * @struct Sp3PositionClockCorrelation
 * @brief SP3 file position and clock correlation information
 * @note The coordinate system of the position is defined in the SP3 header
 */
struct Sp3PositionClockCorrelation {
  int position_x_standard_deviation_mm_ = 0;  //!< Satellite position X standard deviation [mm]
  int position_y_standard_deviation_mm_ = 0;  //!< Satellite position Y standard deviation [mm]
  int position_z_standard_deviation_mm_ = 0;  //!< Satellite position Z standard deviation [mm]
  int clock_standard_deviation_ps_ = 0;       //!< Satellite clock offset standard deviation [ps]
  int x_y_correlation_ = 0;                   //!< Position X-Y correlation [-]
  int x_z_correlation_ = 0;                   //!< Position X-Z correlation [-]
  int x_clock_correlation_ = 0;               //!< Position X - Clock correlation [-]
  int y_z_correlation_ = 0;                   //!< Position Y-Z correlation [-]
  int y_clock_correlation_ = 0;               //!< Position Y - Clock correlation [-]
  int z_clock_correlation_ = 0;               //!< Position Z - Clock correlation [-]
};

/**
 * @struct Sp3VelocityClockRate
 * @brief SP3 file velocity and clock rate information
 * @note The coordinate system of the position is defined in the SP3 header
 */
struct Sp3VelocityClockRate {
  std::string satellite_id_;                           //!< GNSS satellite ID
  math::Vector<3> velocity_dm_s_{0.0};                //!< Satellite velocity [dm/s]
  double clock_rate_ = 0.0;                            //!< Satellite clock offset change rate [-]
  math::Vector<3> velocity_standard_deviation_{0.0};  //!< Satellite position standard deviation [-]
  double clock_rate_standard_deviation_ = 0.0;         //!< Satellite clock offset standard deviation [-]
};

/**
 * @struct Sp3VelocityClockRateCorrelation
 * @brief SP3 file velocity and clock rate correlation information
 * @note The coordinate system of the position is defined in the SP3 header
 */
struct Sp3VelocityClockRateCorrelation {
  int velocity_x_standard_deviation_ = 0;  //!< Satellite velocity X standard deviation [10^-4 mm/s]
  int velocity_y_standard_deviation_ = 0;  //!< Satellite velocity Y standard deviation [10^-4 mm/s]
  int velocity_z_standard_deviation_ = 0;  //!< Satellite velocity Z standard deviation [10^-4 mm/s]
  int clock_rate_standard_deviation_ = 0;  //!< Satellite clock rate offset standard deviation [10^-4 ps]
  int x_y_correlation_ = 0;                //!< Velocity X-Y correlation [-]
  int x_z_correlation_ = 0;                //!< Velocity X-Z correlation [-]
  int x_clock_correlation_ = 0;            //!< Velocity X - Clock correlation [-]
  int y_z_correlation_ = 0;                //!< Velocity Y-Z correlation [-]
  int y_clock_correlation_ = 0;            //!< Velocity Y - Clock correlation [-]
  int z_clock_correlation_ = 0;            //!< Velocity Z - Clock correlation [-]
};

/**
 * @class Sp3FileReader
 * @brief
 */
class Sp3FileReader {
 public:
  /**
   * @fn Sp3FileReader
   * @brief Constructor
   * @param[in] file_name: File name of target SP3 file with directory path
   */
  Sp3FileReader(const std::string file_name);

  // Getter
  // Header information
  inline Sp3Header GetHeader() const { return header_; }
  inline size_t GetNumberOfEpoch() const { return header_.number_of_epoch_; }
  inline size_t GetNumberOfSatellites() const { return header_.number_of_satellites_; }
  inline DateTime GetStartEpochDateTime() const { return header_.start_epoch_; }
  inline GpsTime GetStartEpochGpsTime() const { return header_.start_gps_time_; }
  // Data
  DateTime GetEpochData(const size_t epoch_id) const;
  Sp3PositionClock GetPositionClock(const size_t epoch_id, const size_t satellite_id);
  double GetSatelliteClockOffset(const size_t epoch_id, const size_t satellite_id);
  math::Vector<3> GetSatellitePosition_km(const size_t epoch_id, const size_t satellite_id);

  size_t SearchNearestEpochId(const EpochTime time);

 private:
  Sp3Header header_;             //!< SP3 header information
  std::vector<DateTime> epoch_;  //!< Epoch data list

  // Orbit and clock data (Use as position_clock_[satellite_id][epoch_id])
  std::map<size_t, std::vector<Sp3PositionClock>> position_clock_;                                  //!< Position and Clock data
  std::map<size_t, std::vector<Sp3PositionClockCorrelation>> position_clock_correlation_;           //!< Position and Clock correlation
  std::map<size_t, std::vector<Sp3VelocityClockRate>> velocity_clock_rate_;                         //!< Velocity and Clock rate data
  std::map<size_t, std::vector<Sp3VelocityClockRateCorrelation>> velocity_clock_rate_correlation_;  //!< Velocity and Clock rate correlation

  /**
   * @fn ReadFile
   * @brief Read SP3 file
   * @param[in] file_name: File name of target SP3 file with directory path
   * @return true: File read success, false: File read error
   */
  bool ReadFile(const std::string file_name);
  /**
   * @fn ReadHeader
   * @brief Read SP3 file
   * @param[in] sp3_file: file stream of the SP3 file
   * @return The last line of header. 0 means error is happened.
   */
  size_t ReadHeader(std::ifstream& sp3_file);
  /**
   * @fn DecodePositionClockData
   * @brief Decode position and clock data in SP3 file
   * @param[in] line: Single line data of the SP3 file
   * @return decoded data
   */
  Sp3PositionClock DecodePositionClockData(std::string line);
  /**
   * @fn DecodePositionClockCorrelation
   * @brief Decode position and clock correlation data in SP3 file
   * @param[in] line: Single line data of the SP3 file
   * @return decoded data
   */
  Sp3PositionClockCorrelation DecodePositionClockCorrelation(std::string line);
  /**
   * @fn DecodeVelocityClockRateData
   * @brief Decode velocity and clock rate data in SP3 file
   * @param[in] line: Single line data of the SP3 file
   * @return decoded data
   */
  Sp3VelocityClockRate DecodeVelocityClockRateData(std::string line);
  /**
   * @fn DecodeVelocityClockRateCorrelation
   * @brief Decode velocity and clock rate correlation data in SP3 file
   * @param[in] line: Single line data of the SP3 file
   * @return decoded data
   */
  Sp3VelocityClockRateCorrelation DecodeVelocityClockRateCorrelation(std::string line);
};

#endif  // S2E_LIBRARY_GNSS_SP3_FILE_READER_HPP_
