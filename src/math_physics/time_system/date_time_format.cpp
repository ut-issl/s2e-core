/**
 * @file date_time_format.cpp
 * @brief Class to handle Gregorian date and time format
 */

#define _CRT_SECURE_NO_WARNINGS  // for sscanf

#include "date_time_format.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace s2e::time_system {

DateTime::DateTime(const std::string date_time) {
  sscanf(date_time.c_str(), "%zu/%zu/%zu %zu:%zu:%lf", &year_, &month_, &day_, &hour_, &minute_, &second_);
}

DateTime::DateTime(const EpochTime epoch_time) {
  // No leap second calculation
  const int mday[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
                      31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};  //!< Number of days in a month
  int days, sec, mon, day;

  // leap year if year%4==0 in 1901-2099
  days = (int)(epoch_time.GetTime_s() / 86400);
  sec = (int)(epoch_time.GetTime_s() - (time_t)days * 86400);

  for (day = days % 1461, mon = 0; mon < 48; mon++) {
    if (day >= mday[mon])
      day -= mday[mon];
    else
      break;
  }

  year_ = 1970 + days / 1461 * 4 + mon / 12;
  month_ = mon % 12 + 1;
  day_ = day + 1;
  hour_ = sec / 3600;
  minute_ = sec % 3600 / 60;
  second_ = sec % 60 + epoch_time.GetFraction_s();
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

}  // namespace s2e::time_system
