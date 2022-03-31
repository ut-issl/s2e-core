#ifndef __LOGGER_H__
#define __LOGGER_H__
#define _CRT_SECURE_NO_WARNINGS

#include <fstream>
#include <string>
#include <vector>

#include "ILoggable.h"

class Logger {
 public:
  Logger(const std::string &file_name, const std::string &data_path, const std::string &ini_file_name, const bool enable_inilog, bool enable = true);
  ~Logger(void);

  void Write(std::string log, bool flag = true);
  void AddLoggable(ILoggable *loggable);
  void ClearLoggables();
  void WriteHeaders(bool add_newline = true);
  void WriteValues(bool add_newline = true);
  void WriteNewLine();
  inline bool IsEnabled();
  inline void Enable(bool enable);
  void CopyFileToLogDir(const std::string &ini_file_name);
  inline std::string GetLogPath() const;

 private:
  std::ofstream csv_file_;
  char registered_num_;
  bool is_enabled_;
  bool is_open_;
  std::vector<ILoggable *> loggables_;

  bool is_enabled_inilog_;
  bool is_success_make_dir_ = false;
  std::string directory_path_;
  std::string CreateDirectory(const std::string &data_path, const std::string &time);
  std::string GetFileName(const std::string &path);
};

bool Logger::IsEnabled() { return is_enabled_; }

void Logger::Enable(bool enable) { is_enabled_ = enable; }

std::string Logger::GetLogPath() const { return directory_path_; }
#endif  //__Logger_H__
