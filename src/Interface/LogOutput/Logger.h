/**
 * @file Logger.h
 * @brief Class to manage log output file
 */

#ifndef __LOGGER_H__
#define __LOGGER_H__
#define _CRT_SECURE_NO_WARNINGS

#include <fstream>
#include <string>
#include <vector>

#include "ILoggable.h"

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
   * @param [in] enable_inilog: Enable flag to save ini files
   * @param [in] enable: Enable flag for logging
   */
  Logger(const std::string &file_name, const std::string &data_path, const std::string &ini_file_name, const bool enable_inilog, bool enable = true);
  /**
   * @fn ~Logger
   * @brief Destructor
   */
  ~Logger(void);

  /**
   * @fn Write
   * @brief Write string to the log
   * @param [in] log: Write target
   * @param [in] flag: Enable flag to write
   */
  void Write(std::string log, bool flag = true);
  /**
   * @fn AddLoggable
   * @brief Add a loggable into the log list
   * @param [in] loggable: loggable
   */
  void AddLoggable(ILoggable *loggable);
  /**
   * @fn ClearLoggable
   * @brief Clear the log list
   */
  void ClearLoggables();

  /**
   * @fn WriteHeaders
   * @brief Write all headers in the log list
   * @param add_newline: Add newline or not
   */
  void WriteHeaders(bool add_newline = true);
  /**
   * @fn WriteValues
   * @brief Write all values in the log list
   * @param add_newline: Add newline or not
   */
  void WriteValues(bool add_newline = true);
  /**
   * @fn WriteNewline
   * @brief Write newline
   */
  void WriteNewLine();

  /**
   * @fn IsEnabled
   * @brief Return enable flag of the log
   */
  inline bool IsEnabled();
  /**
   * @fn Enabled
   * @brief Set enable flag of the log
   */
  inline void Enable(bool enable);
  /**
   * @fn CopyFileToLogDir
   * @brief Copy a file (e.g., ini file) into the log directory
   * @param [in] ini_file_name: The path to the target file to copy
   */
  void CopyFileToLogDir(const std::string &ini_file_name);
  /**
   * @fn GetLogPath
   * @brief Return the path to the directory for log files
   */
  inline std::string GetLogPath() const;

 private:
  std::ofstream csv_file_;              //!< CSV file stream
  char registered_num_;                 //!< Number of registered log? (Not used now. TODO: delete?)
  bool is_enabled_;                     //!< Enable flag for logging
  bool is_open_;                        //!< Is the CSV file opened?
  std::vector<ILoggable *> loggables_;  //!< Log list

  bool is_enabled_inilog_;            //!< Enable flag to save ini files
  bool is_success_make_dir_ = false;  //!< Is success making a directory for log files
  std::string directory_path_;        //!< Path to the directory for log files

  /**
   * @fn CreateDirectory
   * @brief Create a directory to store the log files
   * @param [in] data_path: Path to `data` directory
   * @param[in] time: Time stamp (YYYYMMDD_hhmmss)
   * @return Path to the created directory
   */
  std::string CreateDirectory(const std::string &data_path, const std::string &time);

  /**
   * @fn GetFileName
   * @brief Extract the file name from the name with directory path
   * @param [in] path: Directory path including the file name
   * @return The extracted file name
   */
  std::string GetFileName(const std::string &path);
};

bool Logger::IsEnabled() { return is_enabled_; }

void Logger::Enable(bool enable) { is_enabled_ = enable; }

std::string Logger::GetLogPath() const { return directory_path_; }

#endif  //__Logger_H__
