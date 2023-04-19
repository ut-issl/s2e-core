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
IniAccess::IniAccess(const std::string file_path) : file_path_(file_path) {
  // strcpy_s(file_path_char_, (size_t)_countof(file_path_char_), file_path_.c_str());
  strncpy(file_path_char_, file_path_.c_str(), kMaxCharLength);
}
#else
IniAccess::IniAccess(const std::string file_path) : file_path_(file_path), ini_reader_(file_path) {
  strncpy(file_path_char_, file_path_.c_str(), kMaxCharLength);

  std::string ext = ".ini";
  if (file_path_.size() < 4 || !std::equal(std::rbegin(ext), std::rend(ext), std::rbegin(file_path_))) {
    // this is not ini file(csv)
    return;
  }
  if (ini_reader_.ParseError() != 0) {
    std::cerr << "Error reading INI file : " << file_path_ << std::endl;
    std::cerr << "\t error code: " << ini_reader_.ParseError() << std::endl;
    throw std::runtime_error("Error reading INI file");
  }
}
#endif

double IniAccess::ReadDouble(const char* section_name, const char* key_name) {
#ifdef WIN32
  std::stringstream value;
  double temp = 0;

  GetPrivateProfileStringA(section_name, key_name, 0, text_buffer_, kMaxCharLength, file_path_char_);

  value << text_buffer_;  // input string
  value >> temp;          // return as double
  //	std::cout << text_buffer_;

  return temp;
#else
  return ini_reader_.GetReal(section_name, key_name, 0);
#endif
}

int IniAccess::ReadInt(const char* section_name, const char* key_name) {
#ifdef WIN32
  int temp;

  temp = GetPrivateProfileIntA(section_name, key_name, 0, file_path_char_);

  return temp;
#else
  return (int)ini_reader_.GetInteger(section_name, key_name, 0);
#endif
}
bool IniAccess::ReadBoolean(const char* section_name, const char* key_name) {
#ifdef WIN32
  int temp;

  temp = GetPrivateProfileIntA(section_name, key_name, 0, file_path_char_);
  if (temp > 0) {
    return true;
  }
  return false;
#else
  return ini_reader_.GetBoolean(section_name, key_name, false);
#endif
}

void IniAccess::ReadDoubleArray(const char* section_name, const char* key_name, const int id, const int num, double* data) {
  for (int i = 0; i < num; i++) {
    std::stringstream edited_key_name;
    edited_key_name << key_name << id << "(" << i << ")";
    data[i] = ReadDouble(section_name, edited_key_name.str().c_str());
  }
}

void IniAccess::ReadQuaternion(const char* section_name, const char* key_name, libra::Quaternion& data) {
  libra::Quaternion temp;
  double norm = 0.0;

  for (int i = 0; i < 4; i++) {  // Read Quaternion as new format
    std::stringstream edited_key_name;
    edited_key_name << key_name << "_(" << i << ")";
    temp[i] = ReadDouble(section_name, edited_key_name.str().c_str());
    norm += temp[i] * temp[i];
  }
  if (norm == 0.0) {  // If it is not new format, try to read old format
    for (int i = 0; i < 4; i++) {
      std::stringstream edited_key_name;
      edited_key_name << key_name << "(" << i << ")";
      data[i] = ReadDouble(section_name, edited_key_name.str().c_str());
    }
  } else {
    data[0] = temp[0];
    data[1] = temp[1];
    data[2] = temp[2];
    data[3] = temp[3];
  }
}

void IniAccess::ReadChar(const char* section_name, const char* key_name, const int size, char* data) {
#ifdef WIN32
  GetPrivateProfileStringA(section_name, key_name, 0, data, size, file_path_char_);
#else
  std::string string_data = ReadString(section_name, key_name);
  strncpy(data, string_data.c_str(), size);
#endif
}

std::string IniAccess::ReadString(const char* section_name, const char* key_name) {
#ifdef WIN32
  char temp[kMaxCharLength];
  ReadChar(section_name, key_name, kMaxCharLength, temp);
  return std::string(temp);
#else
  std::string value = ini_reader_.GetString(section_name, key_name, "NULL");
  return value;
#endif
}

bool IniAccess::ReadEnable(const char* section_name, const char* key_name) {
  std::string enable_string = ReadString(section_name, key_name);
  if (enable_string.compare("ENABLE") == 0) return true;
  if (enable_string.compare("1") == 0) return true;
  return false;
}

std::vector<std::string> IniAccess::ReadStrVector(const char* section_name, const char* key_name) {
  std::vector<std::string> data;
  char temp[kMaxCharLength];
  unsigned int i = 0;
  while (true) {
    std::stringstream c_name;
    c_name << key_name << "(" << i << ")";
    ReadChar(section_name, c_name.str().c_str(), kMaxCharLength, temp);
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

std::vector<std::string> IniAccess::Split(const std::string& input, const char delimiter) {
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void IniAccess::ReadCsvDouble(std::vector<std::vector<double>>& output_value, const int node_num) {
  std::ifstream ifs(file_path_char_);
  if (!ifs.is_open()) {
    std::cerr << "file open error. filename = " << file_path_char_ << std::endl;
  }
  std::string line;
  int line_num = 0;
  output_value.reserve(node_num);
  while (getline(ifs, line)) {
    std::vector<std::string> string_vector = Split(line, ',');
    std::vector<double> temp;
    temp.reserve(node_num);
    for (size_t i = 0; i < string_vector.size(); i++) {
      temp.push_back(std::stod(string_vector.at(i)));
    }
    output_value.push_back(temp);
    line_num++;
  }
}

void IniAccess::ReadCsvDouble(std::vector<std::vector<double>>& output_value, const int node_num, const int row_header_num,
                              const int column_header_num) {
  std::ifstream ifs(file_path_char_);
  if (!ifs.is_open()) {
    std::cerr << "file open error. filename = " << file_path_char_ << std::endl;
  }
  std::string line;
  int line_num = 0;
  output_value.reserve(node_num);
  while (getline(ifs, line)) {
    if (line_num >= row_header_num) {
      std::vector<std::string> string_vector = Split(line, ',');
      std::vector<double> temp;
      temp.reserve(node_num);
      for (int i = 0; i < (int)string_vector.size(); i++) {
        if (i >= column_header_num) {
          temp.push_back(std::stod(string_vector.at(i)));
        }
      }
      output_value.push_back(temp);
    }
    line_num++;
  }
}

void IniAccess::ReadCsvString(std::vector<std::vector<std::string>>& output_value, const int node_num) {
  std::ifstream ifs(file_path_char_);
  if (!ifs.is_open()) {
    std::cerr << "file open error. filename = " << file_path_char_ << std::endl;
  }
  std::string line;
  int line_num = 0;
  output_value.reserve(node_num);
  while (getline(ifs, line)) {
    std::vector<std::string> temp = Split(line, ',');
    temp.reserve(node_num);
    output_value.push_back(temp);
    line_num++;
  }
}
