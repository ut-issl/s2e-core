/**
 * @file gps_time.cpp
 * @brief A class to define GPS time expression
 */

#include "gps_time.hpp"

namespace time_system {

const DateTime GpsTime::kEpochOfGpsTimeInDateTime_ = DateTime("1980/1/6 00:00:00.0");
const EpochTime GpsTime::kEpochOfGpsTimeInEpochTime_ = EpochTime(kEpochOfGpsTimeInDateTime_);
const EpochTime GpsTime::kLeapSecondAheadFromUtc_ = EpochTime(18, 0);  //!< Leap second ahead from UTC @ May 2023

void GpsTime::CalcGpsWeekTime() {
  EpochTime time_diff = epoch_time_ - kEpochOfGpsTimeInEpochTime_;
  week_ = (size_t)(time_diff.GetTime_s() / kSecondsInWeek_);
  elapsed_time_from_week_s_ = (double)(time_diff.GetTime_s() - week_ * kSecondsInWeek_) + time_diff.GetFraction_s();
}

void GpsTime::CalcEpochTime() {
  size_t integer_time_s = (size_t)elapsed_time_from_week_s_;
  double fraction_s = elapsed_time_from_week_s_ - (double)(integer_time_s);
  time_t time_s = week_ * kSecondsInWeek_ + integer_time_s;
  EpochTime time_diff(time_s, fraction_s);

  epoch_time_ = kEpochOfGpsTimeInEpochTime_ + time_diff;
}

}  // namespace time_system
