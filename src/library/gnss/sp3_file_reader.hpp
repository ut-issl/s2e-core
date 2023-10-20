/**
 * @file sp3_file_reader.hpp
 * @brief A class to read the SP3 (Extended Standard Product 3) format file and provide functions to access the data
 * @note Support version: SP3-d
 *       Ref: https://files.igs.org/pub/data/format/sp3d.pdf?_ga=2.115202830.823990648.1664976786-1792483726.1664976785
 */

#ifndef S2E_LIBRARY_GNSS_SP3_FILE_READER_HPP_
#define S2E_LIBRARY_GNSS_SP3_FILE_READER_HPP_

#include <library/math/vector.hpp>
#include <string>
#include <vector>

/**
 * @enum Sp3Mode
 * @brief Data mode of SP3 file to define including data
 */
enum class Sp3Mode {
  kPosition,  //!< Position and clock data mode
  kVelocity,  //!< Velocity and clock rate data mode
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
};

/**
 * @struct Sp3Header
 * @brief SP3 file header information
 */
typedef struct {
  // 1st line information
  // version -> not implemented yet
  Sp3Mode mode_;  //!< position or velocity
  // Start time: Year, Month, Day, Hour, Minute, Second
  size_t number_of_epoch_ = 0;     //!< Number of epoch in the SP3 file
  std::string used_data_;          //!< Used data to generate the SP3 file
  std::string coordinate_system_;  //!< Coordinate system for the position and velocity data
  Sp3OrbitType orbit_type_;        //!< Orbit type
  std::string agency_name_;        //!< Agency name who generates the SP3 file

  // 2nd line information
  // GPS week
  // Seconds of week
  double epoch_interval_s_ = 900.0;  //!< Epoch interval
  double start_time_mjday_;          // Start time of the orbit data including fractional part [Modified Julian day]

  // 3rd line information
  size_t number_of_satellites_ = 0;  //!< Number of satellites in the SP3 file
  // Used satellite IDs (3rd to 11the line)
  std::vector<std::string> satellite_ids_;  //!< Satellite ID list

  // Accuracy of each satellite (12th to 20th line)
  std::vector<double> satellite_accuracy_;  //!< List of accuracy of each satellite

  // 21st line information
  std::string time_system_;  //!< Time system for the position and velocity data

  // 22nd line information
  // Not important

  // 23rd line information
  double base_number_position_ = 1.25;  //!< Floating point base number used for computing the standard deviations of position and velocity
  double base_number_clock_ = 1.025;    //!< Floating point base number used for computing the standard deviations of clock and clock-rate correction

  // 24th - 26th line information
  // Not important
} Sp3Header;

/**
 * @class Sp3FileReader
 * @brief
 */
class Sp3FileReader {
 public:
  /**
   * @fn Sp3FileReader
   * @brief Constructor
   */
  Sp3FileReader();

 private:
  Sp3Header header_;
};

#endif  // S2E_LIBRARY_GNSS_SP3_FILE_READER_HPP_
