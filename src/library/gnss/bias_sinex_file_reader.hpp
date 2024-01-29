/**
 * @file bias_sinex_file_reader.hpp
 * @brief Read bias SINEX format file
 * @note Ref. https://files.igs.org/pub/data/format/sinex_bias_100.pdf
 */

#ifndef S2E_LIBRARY_BIAS_SINEX_FILE_READER_HPP_
#define S2E_LIBRARY_BIAS_SINEX_FILE_READER_HPP_

#include <string>
#include <vector>

/**
 * @enum BiasIdentifier
 * @brief Bias Identifier
 */
enum class BiasIdentifier {
  kDsb,    //!< Differential Signal Bias
  kIsb,    //!< Ionosphere-free Signal Bias
  kOsb,    //!< Observable-specific Signal Bias
  kError,  //!< Error
};
/**
 * @enum BiasTargetSignal
 * @brief Target signal for bias
 */
enum class BiasTargetSignal {
  kP1P2,   //!< P1 - P2
  kP1C1,   //!< P1 - C1
  kP2C2,   //!< P2 - C2
  kError,  //!< Error
};
/**
 * @enum BiasUnit
 * @brief Unit for bias
 */
enum class BiasUnit {
  kNs,     //!< ns
  kCycle,  //!< cycle
  kError,  //!< Error
};

/**
 * @class BiasSolutionData
 * @brief Bias solution data in bias SINEX file
 */
class BiasSolutionData {
 public:
  /**
   * @fn BiasSolutionData
   * @brief Constructor
   */
  BiasSolutionData() {}
  /**
   * @fn ~BiasSolutionData
   * @brief Destructor
   */
  ~BiasSolutionData() {}

  // Setters
  void SetIdentifier(const std::string identifier);
  inline void SetSatelliteSvnCode(const std::string satellite_svn_code) { satellite_svn_code_ = satellite_svn_code; }
  inline void SetSatelliteNumber(const std::string satellite_number) {
    satellite_number_ = satellite_number;
    // satellite_index_ = ConvertSatelliteNumberToIndex(satellite_number);
  }
  inline void SetStationName(const std::string station_name) { station_name_ = station_name; }
  void SetTargetSignal(const std::string signal1, const std::string signal2);
  void SetUnit(const std::string unit);
  inline void SetBias(const double bias) { bias_ = bias; }
  inline void SetBiasStandardDeviation(const double bias_standard_deviation) { bias_standard_deviation_ = bias_standard_deviation; }
  inline void SetSlope_ns_s(const double slope_ns_s) { slope_ns_s_ = slope_ns_s; }
  inline void SetSlopeStandardDeviation_ns_s(const double slope_standard_deviation_ns_s) {
    slope_standard_deviation_ns_s_ = slope_standard_deviation_ns_s;
  }

  // Getters
  inline BiasIdentifier GetIdentifier() { return identifier_; }
  inline std::string GetSatelliteSvnCode() { return satellite_svn_code_; }
  inline std::string GetStationName() { return station_name_; }
  inline BiasTargetSignal GetTargetSignal() { return target_signal_; }
  inline BiasUnit GetUnit() { return unit_; }
  inline double GetBias() { return bias_; }
  inline double GetBiasStandardDeviation() { return bias_standard_deviation_; }
  inline double GetSlope() { return slope_ns_s_; }
  inline double GetSlopeStandardDeviation() { return slope_standard_deviation_ns_s_; }

 private:
  BiasIdentifier identifier_ = BiasIdentifier::kError;         //!< Bias identifier
  std::string satellite_svn_code_ = "";                        //!< Satellite SVN code
  std::string satellite_number_ = "";                          //!< Satellite number
  std::string station_name_ = "";                              //!< Station name
  BiasTargetSignal target_signal_ = BiasTargetSignal::kError;  //!< Target signal for the bias information
  BiasUnit unit_ = BiasUnit::kError;                           //!< Unit information
  double bias_ = 0.0;                                          //!< Bias [unit is defined by the unit information]
  double bias_standard_deviation_ = 0.0;                       //!< Standard deviation of bias [unit is defined by the unit information
  double slope_ns_s_ = 0.0;                                    //!< Slope parameter [ns/s]
  double slope_standard_deviation_ns_s_ = 0.0;                 //!< Standard deviation of slope parameter [ns/s]
};

/**
 * @class BiasSinexFileReader
 * @brief Read bias SINEX format file
 */
class BiasSinexFileReader {
 public:
  /**
   * @fn BiasSinexFileReader
   * @brief Constructor
   * @param[in] file_name: File path to the bias SINEX file
   */
  BiasSinexFileReader(const std::string file_name) { is_file_read_succeeded_ = ReadFile(file_name); }
  /**
   * @fn ~BiasSinexFileReader
   * @brief Destructor
   */
  ~BiasSinexFileReader() {}

  // Getters
  /**
   * @fn GetFileReadSuccessFlag
   * @return File read success flag
   */
  inline bool GetFileReadSuccessFlag() const { return is_file_read_succeeded_; }
  /**
   * @fn GetNumberOfBiasData
   * @return Number of bias solution data
   */
  inline size_t GetNumberOfBiasData() const { return solution_data_.size(); }
  /**
   * @fn GetFileReadSuccessFlag
   * @param[in] index: Index of bias solution data
   * @return File read success flag
   */
  inline BiasSolutionData GetBiasData(const size_t index) const { return solution_data_[index]; }

 private:
  bool is_file_read_succeeded_;                  //!< File read success flag
  std::vector<BiasSolutionData> solution_data_;  //!< List of solution data

  /**
   * @fn ReadFile
   * @brief Read file data
   * @param[in] file_name: File path to the bias SINEX file
   * @return true: read success, false: read fail
   */
  bool ReadFile(const std::string file_name);

  /**
   * @fn ReadFileReference
   * @brief Read File/Reference data
   * @param[in] bias_sinex_file: bias SINEX file stream
   */
  void ReadFileReference(std::ifstream& bias_sinex_file);
  /**
   * @fn ReadFileComment
   * @brief Read File/Comment data
   * @param[in] bias_sinex_file: bias SINEX file stream
   */
  void ReadFileComment(std::ifstream& bias_sinex_file);
  /**
   * @fn ReadInputAcknowledgments
   * @brief Read Input/Acknowledgments data
   * @param[in] bias_sinex_file: bias SINEX file stream
   */
  void ReadInputAcknowledgments(std::ifstream& bias_sinex_file);
  /**
   * @fn ReadBiasDescription
   * @brief Read Bias/Description data
   * @param[in] bias_sinex_file: bias SINEX file stream
   */
  void ReadBiasDescription(std::ifstream& bias_sinex_file);
  /**
   * @fn ReadBiasReceiverInformation
   * @brief Read Bias/ReceiverInformation data
   * @param[in] bias_sinex_file: bias SINEX file stream
   */
  void ReadBiasReceiverInformation(std::ifstream& bias_sinex_file);
  /**
   * @fn ReadSolution
   * @brief Read Bias/Solution data
   * @param[in] bias_sinex_file: bias SINEX file stream
   */
  void ReadBiasSolution(std::ifstream& bias_sinex_file);
};

#endif  // S2E_LIBRARY_BIAS_SINEX_FILE_READER_HPP_