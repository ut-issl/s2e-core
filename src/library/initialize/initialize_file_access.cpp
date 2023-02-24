/**
 * @file initialize_file_access.cpp
 * @brief Class to read and get parameters for the `ini` format file
 */

#include "initialize_file_access.hpp"

#include <string.h>

#include <algorithm>
#include <cstring>
#include <limits>

#ifdef WIN32
IniAccess::IniAccess(string path) : file_path_(path) {
  // strcpy_s(strPath_, (size_t)_countof(strPath_), file_path_.c_str());
  strncpy(strPath_, file_path_.c_str(), kMaxCharLength);
}
#else
IniAccess::IniAccess(std::string path) : file_path_(path), reader(path) {
  strncpy(strPath_, file_path_.c_str(), kMaxCharLength);

  std::string ext = ".ini";
  if (path.size() < 4 || !std::equal(std::rbegin(ext), std::rend(ext), std::rbegin(path))) {
    // this is not ini file(csv)
    return;
  }
  if (reader.ParseError() != 0) {
    std::cerr << "Error reading INI file : " << path << std::endl;
    std::cerr << "\t error code: " << reader.ParseError() << std::endl;
    throw std::runtime_error("Error reading INI file");
  }
}
#endif

double IniAccess::ReadDouble(const char* section_name, const char* key_name) {
#ifdef WIN32
  stringstream value;
  double temp = 0;

  GetPrivateProfileStringA(section_name, key_name, 0, strText_, 1024, strPath_);

  value << strText_;  // input string
  value >> temp;      // return as double
  //	cout << strText_;

  return temp;
#else
  return reader.GetReal(section_name, key_name, 0);
#endif
}

int IniAccess::ReadInt(const char* section_name, const char* key_name) {
#ifdef WIN32
  int temp;

  temp = GetPrivateProfileIntA(section_name, key_name, 0, strPath_);

  return temp;
#else
  return (int)reader.GetInteger(section_name, key_name, 0);
#endif
}
bool IniAccess::ReadBoolean(const char* section_name, const char* key_name) {
#ifdef WIN32
  int temp;

  temp = GetPrivateProfileIntA(section_name, key_name, 0, strPath_);
  if (temp > 0) {
    return true;
  }
  return false;
#else
  return reader.GetBoolean(section_name, key_name, false);
#endif
}

void IniAccess::ReadDoubleArray(const char* section_name, const char* key_name, int id, int num, double* data) {
  for (int i = 0; i < num; i++) {
    std::stringstream c_name;
    c_name << key_name << id << "(" << i << ")";
    data[i] = ReadDouble(section_name, c_name.str().c_str());
  }
}

void IniAccess::ReadQuaternion(const char* section_name, const char* key_name, libra::Quaternion& data) {
  libra::Quaternion temp;
  double norm = 0.0;

  for (int i = 0; i < 4; i++) {  // Read Quaternion as new format
    std::stringstream c_name;
    c_name << key_name << "_(" << i << ")";
    temp[i] = ReadDouble(section_name, c_name.str().c_str());
    norm += temp[i] * temp[i];
  }
  if (norm == 0.0) {  // If it is not new format, try to read old format
    for (int i = 0; i < 4; i++) {
      std::stringstream c_name;
      c_name << key_name << "(" << i << ")";
      data[i] = ReadDouble(section_name, c_name.str().c_str());
    }
  } else {
    data[0] = temp[0];
    data[1] = temp[1];
    data[2] = temp[2];
    data[3] = temp[3];
  }
}

void IniAccess::ReadChar(const char* section_name, const char* key_name, int size, char* data) {
#ifdef WIN32
  GetPrivateProfileStringA(section_name, key_name, 0, data, size, strPath_);
#else
  std::string sdata = ReadString(section_name, key_name);
  strncpy(data, sdata.c_str(), size);
#endif
}

std::string IniAccess::ReadString(const char* section_name, const char* key_name) {
#ifdef WIN32
  char temp[1024];
  ReadChar(section_name, key_name, 1024, temp);
  return string(temp);
#else
  std::string value = reader.GetString(section_name, key_name, "NULL");
  return value;
#endif
}

bool IniAccess::ReadEnable(const char* section_name, const char* key_name) {
  std::string enablestr = ReadString(section_name, key_name);
  if (enablestr.compare("ENABLE") == 0) return true;
  if (enablestr.compare("1") == 0) return true;
  return false;
}

std::vector<std::string> IniAccess::ReadStrVector(const char* section_name, const char* key_name) {
  const static unsigned int buf_size = 1024;
  std::vector<std::string> data;
  char temp[buf_size];
  unsigned int i = 0;
  while (true) {
    std::stringstream c_name;
    c_name << key_name << "(" << i << ")";
    ReadChar(section_name, c_name.str().c_str(), buf_size, temp);
#ifdef WIN32
    if (temp[0] == NULL) {
#else
    if (!strcmp(temp, "NULL")) {
#endif
      break;
    } else {
      data.push_back(temp);
      i++;
    }
  }
  return data;
}

std::vector<std::string> IniAccess::Split(std::string& input, char delimiter) {
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void IniAccess::ReadCsvDouble(std::vector<std::vector<double>>& doublevec, int node_num) {
  std::ifstream ifs(strPath_);
  if (!ifs.is_open()) {
    std::cerr << "file open error. filename = " << strPath_ << std::endl;
  }
  std::string line;
  int line_num = 0;
  doublevec.reserve(node_num);
  while (getline(ifs, line)) {
    std::vector<std::string> strvec = Split(line, ',');
    std::vector<double> tempdoublevec;
    tempdoublevec.reserve(node_num);
    for (size_t i = 0; i < strvec.size(); i++) {
      tempdoublevec.push_back(std::stod(strvec.at(i)));
    }
    doublevec.push_back(tempdoublevec);
    line_num++;
  }
}

void IniAccess::ReadCsvString(std::vector<std::vector<std::string>>& stringvec, int node_num) {
  std::ifstream ifs(strPath_);
  if (!ifs.is_open()) {
    std::cerr << "file open error. filename = " << strPath_ << std::endl;
  }
  std::string line;
  int line_num = 0;
  stringvec.reserve(node_num);
  while (getline(ifs, line)) {
    std::vector<std::string> tempstrvec = Split(line, ',');
    tempstrvec.reserve(node_num);
    stringvec.push_back(tempstrvec);
    line_num++;
  }
}
