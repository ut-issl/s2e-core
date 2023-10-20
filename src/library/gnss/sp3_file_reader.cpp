/**
 * @file sp3_file_reader.cpp
 * @brief A class to read the SP3 (Extended Standard Product 3) format file and provide functions to access the data
 * @note Support version: SP3-d
 *       Ref: https://files.igs.org/pub/data/format/sp3d.pdf?_ga=2.115202830.823990648.1664976786-1792483726.1664976785
 */

#include "sp3_file_reader.hpp"

#include <fstream>
#include <iostream>

Sp3FileReader::Sp3FileReader(const std::string file_name) { ReadFile(file_name); }

bool Sp3FileReader::ReadFile(const std::string file_name) {
  std::ifstream sp3_file(file_name);
  if (!sp3_file.is_open()) {
    std::cout << "[Warning] SP3 file not found: " << file_name << std::endl;
    return false;
  }

  size_t line_number = ReadHeader(sp3_file);
  if (line_number == 0) return false;

  sp3_file.close();
  return true;
}

size_t Sp3FileReader::ReadHeader(std::ifstream& sp3_file) {
  size_t line_number = 0;
  std::string line;

  // 1st line
  line_number++;
  std::getline(sp3_file, line);
  if (line.find("#d") != 0) {
    std::cout << "[Warning] SP3 file version is not supported: " << line << std::endl;
    return 0;
  }

  return 0;
}
