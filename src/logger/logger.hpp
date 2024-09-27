/**
 * @file logger.hpp
 * @brief Class to manage log output file
 */

#ifndef S2E_LIBRARY_LOGGER_LOGGER_HPP_
#define S2E_LIBRARY_LOGGER_LOGGER_HPP_

#define _CRT_SECURE_NO_WARNINGS

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "loggable.hpp"

namespace s2e::logger {

/**
 * @class Logger
 * @brief Class to manage log output file
 */
class Logger {
 public:
  /**
   * @fn Logger
   * @brief Constructor
   * @param [in] file_name: File name of the log output
   * @param [in] data_path: Path to `data` directory
   * @param [in] ini_file_name: Initialize file name
   * @param [in] is_ini_save_enabled: Enable flag to save ini files
   * @param [in] is_enabled: Enable flag for logging
   */
  Logger(const std::string &file_name, const std::filesystem::path &data_path, const std::filesystem::path &ini_file_name,
         const bool is_ini_save_enabled, const bool is_enabled = true);
  /**
   * @fn ~Logger
   * @brief Destructor
   */
  ~Logger(void);

  /**
   * @fn AddLogList
   * @brief Add a loggable into the log list
   * @param [in] loggable: loggable
   */
  void AddLogList(ILoggable *loggable);
  /**
   * @fn ClearLogList
   * @brief Clear the log list
   */
  void ClearLogList();

  /**
   * @fn WriteHeaders
   * @brief Write all headers in the log list
   * @param add_newline: Add newline or not
   */
  void WriteHeaders(const bool add_newline = true);
  /**
   * @fn WriteValues
   * @brief Write all values in the log list
   * @param add_newline: Add newline or not
   */
  void WriteValues(const bool add_newline = true);

  /**
   * @fn Enabled
   * @brief Set enable flag of the log
   */
  inline void Enable(const bool enable) { is_enabled_ = enable; }
  /**
   * @fn CopyFileToLogDirectory
   * @brief Copy a file (e.g., ini file) into the log directory
   * @param [in] ini_file_name: The path to the target file to copy
   */
  void CopyFileToLogDirectory(const std::filesystem::path &ini_file_name);

  // Getter
  /**
   * @fn IsEnabled
   * @brief Return enable flag of the log
   */
  inline bool IsEnabled() const { return is_enabled_; }
  /**
   * @fn GetLogPath
   * @brief Return the path to the directory for log files
   */
  inline std::filesystem::path GetLogPath() const { return directory_path_; }

 private:
  std::ofstream csv_file_;             //!< CSV file stream
  bool is_enabled_;                    //!< Enable flag for logging
  bool is_file_opened_;                //!< Is the CSV file opened?
  static bool is_directory_created_;   //!< Is the log output directory is created in the scenario
  std::vector<ILoggable *> log_list_;  //!< Log list

  bool is_ini_save_enabled_;              //!< Enable flag to save ini files
  std::filesystem::path directory_path_;  //!< Path to the directory for log files

  /**
   * @fn Write
   * @brief Write string to the log
   * @param [in] log: Write target
   * @param [in] flag: Enable flag to write
   */
  void Write(const std::string log, const bool flag = true);

  /**
   * @fn WriteNewline
   * @brief Write newline
   */
  void WriteNewLine();

  /**
   * @fn CreateDirectory
   * @brief Create a directory to store the log files
   * @param [in] data_path: Path to `data` directory
   * @param[in] time: Time stamp (YYYYMMDD_hhmmss)
   * @return Path to the created directory
   */
  std::filesystem::path CreateDirectory(const std::filesystem::path &data_path, const std::string &time);
};

} // namespace s2e::logger

#endif  // S2E_LIBRARY_LOGGER_LOGGER_HPP_
