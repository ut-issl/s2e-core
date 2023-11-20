/**
 * @file gps_time.cpp
 * @brief A class to define GPS time expression
 */

#include "gps_time.hpp"

const EpochTime GpsTime::epoch_of_gps_time_in_epoch_time_ = EpochTime(DateTime("1980/1/6 00:00:00.0"));

void GpsTime::CalcGpsWeekTime() {
  EpochTime time_diff = epoch_time_ - epoch_of_gps_time_in_epoch_time_;
  week_ = (size_t)(time_diff.GetTime_s() / seconds_in_week_);
  elapsed_time_from_week_s_ = (double)(time_diff.GetTime_s() - week_ * seconds_in_week_) + time_diff.GetFraction_s();
}
