/**
 * @file date_time_format.cpp
 * @brief Class to handle Gregorian date and time format
 */

#define _CRT_SECURE_NO_WARNINGS  // for sscanf

#include "date_time_format.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

DateTime::DateTime(const std::string date_time) {
  sscanf(date_time.c_str(), "%zu/%zu/%zu %zu:%zu:%lf", &year_, &month_, &day_, &hour_, &minute_, &second_);
}

std::string DateTime::GetAsString() const {
  std::stringstream stream;
  stream << std::setw(4) << std::setfill('0') << (int)year_ << "/";
  stream << std::setw(2) << std::setfill('0') << (int)month_ << "/";
  stream << std::setw(2) << std::setfill('0') << (int)day_ << " ";

  stream << std::setw(2) << std::setfill('0') << (int)hour_ << ":";
  stream << std::setw(2) << std::setfill('0') << (int)minute_ << ":";
  stream << second_;
  std::string output = stream.str();
  return output;
}
