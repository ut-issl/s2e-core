/**
 * @file initialize_file_access.cpp
 * @brief Class to read and get parameters for the `ini` format file
 */

#include "initialize_file_access.hpp"

#include <string.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <regex>

#include "../utilities/macros.hpp"

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

std::vector<unsigned char> IniAccess::ReadVectorUnsignedChar(const char* section_name, const char* key_name, const size_t num) {
  std::vector<unsigned char> data;
  for (size_t i = 0; i < num; i++) {
    std::stringstream edited_key_name;
    edited_key_name << key_name << "(" << i << ")";
    data.push_back((unsigned char)ReadInt(section_name, edited_key_name.str().c_str()));
  }
  return data;
}

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
  UNUSED(text_buffer_);
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

std::vector<int> IniAccess::ReadVectorInt(const char* section_name, const char* key_name, const size_t num) {
  std::vector<int> data;
  for (size_t i = 0; i < num; i++) {
    std::stringstream edited_key_name;
    edited_key_name << key_name << "(" << i << ")";
    data.push_back(ReadInt(section_name, edited_key_name.str().c_str()));
  }
  return data;
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

std::vector<double> IniAccess::ReadVectorDouble(const char* section_name, const char* key_name, const size_t num) {
  std::vector<double> data;
  for (size_t i = 0; i < num; i++) {
    std::stringstream edited_key_name;
    edited_key_name << key_name << "(" << i << ")";
    data.push_back(ReadDouble(section_name, edited_key_name.str().c_str()));
  }
  return data;
}

void IniAccess::ReadQuaternion(const char* section_name, const char* key_name, math::Quaternion& data) {
  math::Quaternion temp;
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
  std::string value;
#ifdef WIN32
  char temp[kMaxCharLength];
  ReadChar(section_name, key_name, kMaxCharLength, temp);
  value = std::string(temp);
#else
  value = ini_reader_.GetString(section_name, key_name, "NULL");
#endif
  // Special characters
  // Inline comments
  std::regex inline_comment_pattern("\\s*//.*");
  value = std::regex_replace(value, inline_comment_pattern, "");
  // INI_FILE_DIR
  std::string ini_path = INI_FILE_DIR_FROM_EXE;
  value = std::regex_replace(value, std::regex("INI_FILE_DIR_FROM_EXE"), ini_path);
  // EXT_LIB_DIR
  std::string ext_lib_path = EXT_LIB_DIR_FROM_EXE;
  value = std::regex_replace(value, std::regex("EXT_LIB_DIR_FROM_EXE"), ext_lib_path);
  // CORE_DIR
  std::string s2e_core_path = CORE_DIR_FROM_EXE;
  value = std::regex_replace(value, std::regex("CORE_DIR_FROM_EXE"), s2e_core_path);

  return value;
}

std::vector<std::string> IniAccess::ReadVectorString(const char* section_name, const char* key_name, const size_t num) {
  std::vector<std::string> data;
  for (size_t i = 0; i < num; i++) {
    std::stringstream edited_key_name;
    edited_key_name << key_name << "(" << i << ")";
    data.push_back(ReadString(section_name, edited_key_name.str().c_str()));
  }
  return data;
}

bool IniAccess::ReadEnable(const char* section_name, const char* key_name) {
  std::string enable_string = ReadString(section_name, key_name);
  if (enable_string.compare("ENABLE") == 0) return true;
  if (enable_string.compare("1") == 0) return true;
  return false;
}

std::vector<std::string> IniAccess::ReadStrVector(const char* section_name, const char* key_name) {
  std::vector<std::string> data;
  std::string temp;
  unsigned int i = 0;
  while (true) {
    std::stringstream c_name;
    c_name << key_name << "(" << i << ")";
    temp = ReadString(section_name, c_name.str().c_str());
#ifdef WIN32
    if (temp.c_str()[0] == NULL) {
#else
    if (!strcmp(temp.c_str(), "NULL")) {
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

void IniAccess::ReadCsvDouble(std::vector<std::vector<double>>& output_value, const size_t node_num) {
  std::ifstream ifs(file_path_char_);
  if (!ifs.is_open()) {
    std::cerr << "file open error. filename = " << file_path_char_ << std::endl;
  }
  std::string line;

  output_value.reserve(node_num);
  while (getline(ifs, line)) {
    std::vector<std::string> string_vector = Split(line, ',');
    std::vector<double> temp;
    temp.reserve(node_num);
    for (size_t i = 0; i < string_vector.size(); i++) {
      temp.push_back(std::stod(string_vector.at(i)));
    }
    output_value.push_back(temp);
  }
}

void IniAccess::ReadCsvDoubleWithHeader(std::vector<std::vector<double>>& output_value, const size_t node_num, const size_t row_header_num,
                                        const size_t column_header_num) {
  std::ifstream ifs(file_path_char_);
  if (!ifs.is_open()) {
    std::cerr << "file open error. filename = " << file_path_char_ << std::endl;
  }
  std::string line;
  size_t line_num = 0;
  output_value.reserve(node_num);
  while (getline(ifs, line)) {
    if (line_num >= row_header_num) {
      std::vector<std::string> string_vector = Split(line, ',');
      std::vector<double> temp;
      temp.reserve(node_num);
      for (size_t i = 0; i < string_vector.size(); i++) {
        if (i >= column_header_num) {
          temp.push_back(std::stod(string_vector.at(i)));
        }
      }
      output_value.push_back(temp);
    }
    line_num++;
  }
}

void IniAccess::ReadCsvString(std::vector<std::vector<std::string>>& output_value, const size_t node_num) {
  std::ifstream ifs(file_path_char_);
  if (!ifs.is_open()) {
    std::cerr << "file open error. filename = " << file_path_char_ << std::endl;
  }
  std::string line;
  output_value.reserve(node_num);
  while (getline(ifs, line)) {
    std::vector<std::string> temp = Split(line, ',');
    temp.reserve(node_num);
    output_value.push_back(temp);
  }
}
