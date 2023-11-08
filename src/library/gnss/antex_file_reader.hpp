/**
 * @file antex_reader.hpp
 * @brief Read ANTEX format file
 * @note Ref. https://files.igs.org/pub/data/format/antex14.txt
 */

#ifndef S2E_LIBRARY_ANTEX_FILE_READER_HPP_
#define S2E_LIBRARY_ANTEX_FILE_READER_HPP_

#include <stdint.h>

#include <library/math/vector.hpp>
#include <library/time_system/date_time_format.hpp>
#include <map>
#include <string>
#include <vector>

/**
 * @class AntexGridDefinition
 * @brief grid data definition in ANTEX file
 * @note TODO: support azimuth depending data
 */
class AntexGridDefinition {
 public:
  /**
   * @fn AntexGridDefinition
   * @brief Constructor
   * @param[in] start_deg: Start azimuth angle [deg]
   * @param[in] end_deg: End azimuth angle [deg]
   * @param[in] step_deg: Step azimuth angle [deg]
   */
  AntexGridDefinition(double start_deg = 0.0, double end_deg = 90.0, double step_deg = 10.0)
      : start_deg_(start_deg), end_deg_(end_deg), step_deg_(step_deg) {
    number_of_grid_ = size_t((end_deg_ - start_deg_) / step_deg_) + 1;
  }
  /**
   * @fn ~AntexGridDefinition
   * @brief Destructor
   */
  ~AntexGridDefinition() {}

  // Getter
  inline double GetStart_deg() const { return start_deg_; }
  inline double GetEnd_deg() const { return end_deg_; }
  inline double GetStep_deg() const { return step_deg_; }
  inline double GetNumberOfGrid() const { return number_of_grid_; }

 private:
  double start_deg_;       //!< Grid start value (ZEN1) [deg]
  double end_deg_;         //!< Grid end value (ZEN2) [deg]
  double step_deg_;        //!< Grid step value (DZEN) [deg]
  size_t number_of_grid_;  //!< Number of grid
};

/**
 * @class AntexPhaseCenterData
 * @brief Phase center data in ANTEX file
 * @note The frame definition of the offset is X, Y, Z in IGS-specific frame for GNSS satellite and North, East, Up for receivers
 * @note TODO: support azimuth depending data
 */
class AntexPhaseCenterData {
 public:
  /**
   * @fn ~AntexPhaseCenterData
   * @brief Constructor
   */
  AntexPhaseCenterData() {}
  /**
   * @fn ~AntexPhaseCenterData
   * @brief Destructor
   */
  ~AntexPhaseCenterData() {}

  // Setter
  inline void SetFrequencyName(const std::string frequency_name) { frequency_name_ = frequency_name; }
  inline void SetPhaseCenterOffset_mm(const libra::Vector<3> phase_center_offset_mm) { phase_center_offset_mm_ = phase_center_offset_mm; }
  inline void SetGridInformation(const AntexGridDefinition grid_information) { grid_information_ = grid_information; }
  inline void SetPhaseCenterVariationParameters_mm(const std::vector<double> phase_center_variation_parameters_mm) {
    phase_center_variation_parameters_mm_ = phase_center_variation_parameters_mm;
  }

  // Getter
  inline std::string GetFrequencyName() const { return frequency_name_; }
  inline libra::Vector<3> GetPhaseCenterOffset_mm() const { return phase_center_offset_mm_; }
  inline AntexGridDefinition GetGridInformation() const { return grid_information_; }
  inline std::vector<double> GetPhaseCenterVariationParameters_mm() const { return phase_center_variation_parameters_mm_; }

 private:
  std::string frequency_name_ = "";                           //!< Frequency name
  libra::Vector<3> phase_center_offset_mm_{0.0};              //!< Phase center offset for each frequency [mm]
  AntexGridDefinition grid_information_;                      //!< Grid information
  std::vector<double> phase_center_variation_parameters_mm_;  //!< Phase center variation for each frequency [mm]
};

/**
 * @class AntexSatelliteData
 * @brief
 */
class AntexSatelliteData {
 public:
  AntexSatelliteData() {}
  ~AntexSatelliteData() {}

  // Setter
  inline void SetAntennaType(const std::string antenna_type) { antenna_type_ = antenna_type; }
  inline void SetSerialNumber(const std::string serial_number) { serial_number_ = serial_number; }
  inline void SetValidStartTime(const DateTime valid_start_time) { valid_start_time_ = valid_start_time; };
  inline void SetValidEndTime(const DateTime valid_end_time) { valid_end_time_ = valid_end_time; };
  inline void SetNumberOfFrequency(const size_t number_of_frequency) { number_of_frequency_ = number_of_frequency; };
  inline void SetPhaseCenterData(const std::vector<AntexPhaseCenterData> phase_center_data) { phase_center_data_ = phase_center_data; };

  // Getter
  inline std::string GetAntennaType() const { return antenna_type_; }
  inline std::string GetSerialNumber() const { return serial_number_; }
  inline DateTime GetValidStartTime() const { return valid_start_time_; };
  inline DateTime GetValidEndTime() const { return valid_end_time_; };
  inline size_t GetNumberOfFrequency() const { return number_of_frequency_; };
  inline AntexPhaseCenterData GetPhaseCenterData(const size_t frequency_index) const { return phase_center_data_[frequency_index]; };

 private:
  std::string antenna_type_;                             //!< Antenna type
  std::string serial_number_;                            //!< Serial number or satellite code
  DateTime valid_start_time_;                            //!< Valid start time
  DateTime valid_end_time_;                              //!< Valid end time
  size_t number_of_frequency_ = 1;                       //!< Number of frequency
  std::vector<AntexPhaseCenterData> phase_center_data_;  //!< Phase center data for each frequency
};

/**
 * @class AntexFileReader
 * @brief Read ANTEX format file
 */
class AntexFileReader {
 public:
  /**
   * @fn AntexFileReader
   * @brief Constructor
   * @param[in] file_name: File path to the ANTEX file
   */
  AntexFileReader(std::string file_name) { is_file_read_succeeded_ = ReadFile(file_name); }
  /**
   * @fn ~AntexFileReader
   * @brief Destructor
   */
  ~AntexFileReader() {}

  // Getters
  inline bool GetFileReadSuccessFlag() const { return is_file_read_succeeded_; }
  inline size_t GetNumberOfSatelliteData() const { return antex_satellite_data_.size(); }
  inline std::vector<AntexSatelliteData> GetAntexSatelliteData(const size_t satellite_index) const {
    return antex_satellite_data_.at(satellite_index);
  };

 private:
  bool is_file_read_succeeded_;                                             //!< File read success flag
  std::map<size_t, std::vector<AntexSatelliteData>> antex_satellite_data_;  //!< ANTEX data list for GNSS satellite
  // TODO: Implement data for Receivers

  /**
   * @fn ReadFile
   * @brief Read file data
   * @param[in] file_name: File path to the ANTEX file
   * @return true: read success, false: read fail
   */
  bool ReadFile(const std::string file_name);

  /**
   * @fn ReadAntexData
   * @brief Read ANTEX body data
   * @param[in] antex_file: ANTEX file stream
   */
  void ReadAntexData(std::ifstream& antex_file);
  /**
   * @fn ReadAntexSatelliteData
   * @brief Read ANTEX body data for GNSS satellite
   * @param[in] antex_file: ANTEX file stream
   * @return ANTEX data for GNSS satellite
   */
  AntexSatelliteData ReadAntexSatelliteData(std::ifstream& antex_file);
  /**
   * @fn ReadPhaseCenterData
   * @brief Read phase center data
   * @param[in] antex_file: ANTEX file stream
   * @param[in] grid_size: Grid size
   * @return ANTEX phase center data
   */
  AntexPhaseCenterData ReadPhaseCenterData(std::ifstream& antex_file, const AntexGridDefinition grid_information);
  /**
   * @fn ReadDateTime
   * @brief Read date time information in ANTEX file
   * @param[in] line: A single line in ANTEX file
   * @return Read date time
   */
  DateTime ReadDateTime(std::string line);
};

#endif  // S2E_LIBRARY_ANTEX_FILE_READER_HPP_
