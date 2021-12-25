#ifndef __LOGGER_H__
#define __LOGGER_H__
#define _CRT_SECURE_NO_WARNINGS

using namespace std;

#include <string>
#include <fstream>
#include <vector>

#include "ILoggable.h"

class Logger
{
public:
  Logger(const string &file_name, const string &data_path, const string &ini_file_name, const bool enable_inilog, bool enable = true);
  ~Logger(void);

  void Write(string log, bool flag = true);
  void AddLoggable(ILoggable* loggable);
  void ClearLoggables();
  void WriteHeaders(bool add_newline = true);
  void WriteValues(bool add_newline = true);
  void WriteNewLine();
  inline bool IsEnabled();
  inline void Enable(bool enable);
  void CopyFileToLogDir(const string &ini_file_name);
  inline string GetLogPath() const;

private:
  ofstream csv_file_;
  char registered_num_;
  bool is_enabled_;
  bool is_open_;
  vector<ILoggable*> loggables_;

  bool is_enabled_inilog_;
  bool is_success_make_dir_ = false;
  string directory_path_;
  string CreateDirectory(const string &data_path, const string & time);
  string GetFileName(const string &path);
};

bool Logger::IsEnabled()
{
  return is_enabled_;
}

void Logger::Enable(bool enable)
{
  is_enabled_ = enable;
}

string Logger::GetLogPath() const
{
  return directory_path_;
}
#endif //__Logger_H__
