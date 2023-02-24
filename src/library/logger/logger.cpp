/**
 * @file logger.cpp
 * @brief Class to manage log output file
 */

#include "logger.hpp"

#include <ctime>
#include <sstream>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

std::vector<ILoggable *> loggables_;

Logger::Logger(const std::string &file_name, const std::string &data_path, const std::string &ini_file_name, const bool enable_inilog, bool enable) {
  is_enabled_ = enable;
  is_open_ = false;
  is_enabled_inilog_ = enable_inilog;

  // Get current time to append it to the filename
  time_t timer = time(NULL);
  struct tm *now;
  now = localtime(&timer);
  char start_time_c[64];
  strftime(start_time_c, 64, "%y%m%d_%H%M%S", now);

  // Create directory
  if (is_enabled_inilog_ == true)
    directory_path_ = CreateDirectory(data_path, start_time_c);
  else
    directory_path_ = data_path;
  // Create File
  std::stringstream file_path;
  file_path << directory_path_ << start_time_c << "_" << file_name;
  if (is_enabled_) {
    csv_file_.open(file_path.str());
    is_open_ = csv_file_.is_open();
    if (!is_open_) std::cerr << "Error opening log file: " << file_path.str() << std::endl;
  }

  // Copy SimBase.ini
  CopyFileToLogDir(ini_file_name);
}

Logger::~Logger(void) {
  if (is_open_) {
    csv_file_.close();
  }
}

void Logger::WriteHeaders(bool add_newline) {
  for (auto itr = loggables_.begin(); itr != loggables_.end(); ++itr) {
    if (!((*itr)->is_log_enabled_)) continue;
    Write((*itr)->GetLogHeader());
  }
  if (add_newline) WriteNewLine();
}

void Logger::WriteValues(bool add_newline) {
  for (auto itr = loggables_.begin(); itr != loggables_.end(); ++itr) {
    if (!((*itr)->is_log_enabled_)) continue;
    Write((*itr)->GetLogValue());
  }
  if (add_newline) WriteNewLine();
}

void Logger::WriteNewLine() { Write("\n"); }

void Logger::Write(std::string log, bool flag) {
  if (flag && is_enabled_) {
    csv_file_ << log;
  }
}

void Logger::AddLoggable(ILoggable *loggable) { loggables_.push_back(loggable); }

void Logger::ClearLoggables() { loggables_.clear(); }

std::string Logger::CreateDirectory(const std::string &data_path, const std::string &time) {
  std::string directory_path_tmp_ = data_path + "/logs_" + time + "/";
  // Make directory
  int rtn_mkdir = 0;
#ifdef WIN32
  rtn_mkdir = _mkdir(directory_path_tmp_.c_str());
#else
  rtn_mkdir = mkdir(directory_path_tmp_.c_str(), 0777);
#endif
  if (rtn_mkdir == 0) {
  } else {
    std::cerr << "Error making directory: " << directory_path_tmp_ << std::endl;
    return data_path;
  }
  return directory_path_tmp_;
}

void Logger::CopyFileToLogDir(const std::string &ini_file_name) {
  using std::ios;

  if (is_enabled_inilog_ == false) return;
  // Copy files to the directory
  std::string file_name = GetFileName(ini_file_name);
  std::string to_file_name = directory_path_ + file_name;
  std::ifstream is(ini_file_name, ios::in | ios::binary);
  std::ofstream os(to_file_name, ios::out | ios::binary);
  os << is.rdbuf();

  return;
}

std::string Logger::GetFileName(const std::string &path) {
  size_t pos1;

  pos1 = path.rfind('\\');
  if (pos1 != std::string::npos) {
    return path.substr(pos1 + 1, path.size() - pos1 - 1);
  }

  pos1 = path.rfind('/');
  if (pos1 != std::string::npos) {
    return path.substr(pos1 + 1, path.size() - pos1 - 1);
  }

  return path;
}
