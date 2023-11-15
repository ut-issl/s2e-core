/**
 * @file epoch_time.cpp
 * @brief A class to handle time like UNIX time with fractions
 */

#include "epoch_time.hpp"

EpochTime::EpochTime(const DateTime date_time) {
  // No leap second calculation
  const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};  //!< Day of Year for the 1st day of each month
  // Parse Calender
  const size_t year = date_time.GetYear();
  const size_t month = date_time.GetMonth();
  const size_t day = date_time.GetDay();
  const size_t hour = date_time.GetHour();
  const size_t minute = date_time.GetMinute();
  const size_t second = (size_t)std::floor(date_time.GetSecond());

  // TODO: assertion
  if (year < 1970 || month < 1 || 12 < month) return;
  if (day < 1 || 32 <= day) return;
  if (60 <= minute) return;
  if (60 <= second) return;

  // leap year if year%4==0 in 1901-2099
  int days = (year - 1970) * 365 + (year - 1969) / 4 + doy[month - 1] + day - 2 + (year % 4 == 0 && month >= 3 ? 1 : 0);
  time_s_ = (time_t)days * 86400 + hour * 3600 + minute * 60 + second;
  fraction_s_ = date_time.GetSecond() - (double)second;
}

EpochTime TimeAdd(const EpochTime& left_side, const EpochTime& right_side) {
  time_t time_s = left_side.GetTime_s() + right_side.GetTime_s();
  double fraction_s = left_side.GetFraction_s() + right_side.GetFraction_s();
  if (fraction_s > 1.0) {
    fraction_s -= 1.0;
    time_s += 1;
  }
  EpochTime result(time_s, fraction_s);
  return result;
}

EpochTime TimeSubtract(const EpochTime& left_side, const EpochTime& right_side) {
  time_t time_s = left_side.GetTime_s() - right_side.GetTime_s();  // TODO: assertion for negative value
  double fraction_s = left_side.GetFraction_s() - right_side.GetFraction_s();
  if (fraction_s < 0.0) {
    fraction_s += 1.0;
    time_s -= 1;
  }
  EpochTime result(time_s, fraction_s);
  return result;
}