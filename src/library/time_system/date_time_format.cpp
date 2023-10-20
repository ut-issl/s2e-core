/**
 * @file date_time_format.cpp
 * @brief
 */

#include "date_time_format.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

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
