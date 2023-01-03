#include "Logger.h"

#include <ctime>
#include <sstream>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

std::vector<ILoggable *> loggables_;

// file_name: "default.csv", log_root_dir: "../../data/logs/", ini_file_name: "../../data/ini/ISSL6U_SimBase.ini", enable_inilog: true, enable: true
Logger::Logger(const std::string &file_name, const std::string &log_root_dir, const std::string &ini_file_name, const bool enable_inilog, bool enable,
               const std::string &sim_name) {
  is_enabled_ = enable;
  is_open_ = false;
  is_enabled_inilog_ = enable_inilog;

  // If sim_name is not specified, use current time
  std::string sim_name_tmp;
  if (sim_name == "") {
    time_t timer = time(NULL);
    struct tm *now;
    now = localtime(&timer);
    char start_time_c[64];
    strftime(start_time_c, 64, "%y%m%d_%H%M%S", now);
    sim_name_tmp = start_time_c;
  } else {
    sim_name_tmp = sim_name;
  }

  // Create directory
  if (is_enabled_inilog_ == true)
    directory_path_ = CreateDirectory(log_root_dir, sim_name_tmp);
  else
    directory_path_ = log_root_dir;
  // Create File
  std::stringstream file_path;
  file_path << directory_path_ << sim_name_tmp << "_" << file_name;
  if (is_enabled_) {
    csv_file_.open(file_path.str());
    is_open_ = csv_file_.is_open();
    if (!is_open_) std::cerr << "Error opening log file: " << file_path.str() << std::endl;
  }
  registered_num_ = 0;

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
    if (!((*itr)->IsLogEnabled)) continue;
    Write((*itr)->GetLogHeader());
  }
  if (add_newline) WriteNewLine();
}

void Logger::WriteValues(bool add_newline) {
  for (auto itr = loggables_.begin(); itr != loggables_.end(); ++itr) {
    if (!((*itr)->IsLogEnabled)) continue;
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

std::string Logger::CreateDirectory(const std::string &log_root_dir, const std::string &sim_name) {
  std::string directory_path_tmp_ = log_root_dir + "/logs_" + sim_name + "/";
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
    return log_root_dir;
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
