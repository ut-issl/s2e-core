/**
 * @file logger.cpp
 * @brief Class to manage log output file
 */

#include "logger.hpp"

#include <ctime>
#include <sstream>

namespace s2e::logger {

std::vector<ILoggable *> log_list_;
bool Logger::is_directory_created_ = false;

namespace fs = std::filesystem;

Logger::Logger(const std::string &file_name, const fs::path &data_path, const fs::path &ini_file_name, const bool is_ini_save_enabled,
               const bool is_enabled)
    : is_enabled_(is_enabled), is_ini_save_enabled_(is_ini_save_enabled) {
  is_file_opened_ = false;
  if (is_enabled_ == false) return;

  // Set current time to filename prefix
  time_t timer = time(NULL);
  struct tm *now;
  now = localtime(&timer);
  char start_time_c[64];
  strftime(start_time_c, 64, "%y%m%d_%H%M%S", now);

  const auto file_prefix = std::string(start_time_c) + "_";

  // Create directory
  if (is_ini_save_enabled_ == true || is_directory_created_ == false) {
    directory_path_ = CreateDirectory(data_path, start_time_c);
  } else {
    directory_path_ = data_path;
  }

  // Create File
  fs::path file_path = directory_path_ / (file_prefix + file_name);
  if (is_enabled_) {
    csv_file_.open(file_path.string());
    is_file_opened_ = csv_file_.is_open();
    if (!is_file_opened_) std::cerr << "Error opening log file: " << file_path << std::endl;
  }

  // Copy SimBase.ini
  CopyFileToLogDirectory(ini_file_name);
}

Logger::~Logger(void) {
  if (is_file_opened_) {
    csv_file_.close();
  }
}

void Logger::WriteHeaders(const bool add_newline) {
  for (auto itr = log_list_.begin(); itr != log_list_.end(); ++itr) {
    if (!((*itr)->is_log_enabled_)) continue;
    Write((*itr)->GetLogHeader());
  }
  if (add_newline) WriteNewLine();
}

void Logger::WriteValues(const bool add_newline) {
  for (auto itr = log_list_.begin(); itr != log_list_.end(); ++itr) {
    if (!((*itr)->is_log_enabled_)) continue;
    Write((*itr)->GetLogValue());
  }
  if (add_newline) WriteNewLine();
}

void Logger::WriteNewLine() { Write("\n"); }

void Logger::Write(const std::string log, const bool flag) {
  if (flag && is_enabled_) {
    csv_file_ << log;
  }
}

void Logger::AddLogList(ILoggable *loggable) { log_list_.push_back(loggable); }

void Logger::ClearLogList() { log_list_.clear(); }

fs::path Logger::CreateDirectory(const fs::path &data_path, const std::string &time) {
  fs::path log_dir_ = data_path;
  log_dir_.append(std::string("logs_") + time);

  fs::create_directories(log_dir_);

  return log_dir_;
}

void Logger::CopyFileToLogDirectory(const fs::path &ini_file_name) {
  if (is_ini_save_enabled_ == false) return;
  // Copy files to the directory
  fs::path to_file_name = directory_path_ / ini_file_name.filename();

  if (fs::exists(to_file_name)) {
    std::cout << "File " << to_file_name << " already exists. Skip copy from" << ini_file_name << std::endl;
    return;
  }

  fs::copy_file(ini_file_name, to_file_name);
  return;
}

}  // namespace s2e::logger
