/**
 * @file gps_time.hpp
 * @brief A class to define GPS time expression
 */

#ifndef S2E_LIBRARY_TIME_SYSTEM_GPS_TIME_HPP_
#define S2E_LIBRARY_TIME_SYSTEM_GPS_TIME_HPP_

#include <cstddef>

#include "date_time_format.hpp"
#include "epoch_time.hpp"

namespace s2e::time_system {

/**
 * @class GpsTime
 * @brief A class to define GPS time expression
 * @note GPS time is 18 sec ahead from UTC
 */
class GpsTime {
 public:
  /**
   * @fn GpsTime
   * @brief Constructor initialized with week and second
   */
  GpsTime(const size_t week = 0, const double elapsed_time_from_week_s = 0.0) : week_(week), elapsed_time_from_week_s_(elapsed_time_from_week_s) {
    CalcEpochTime();
    date_time_ = DateTime(epoch_time_);
  }
  /**
   * @fn GpsTime
   * @brief Constructor initialized with epoch time expression
   */
  GpsTime(const EpochTime epoch_time) : date_time_(DateTime(epoch_time)), epoch_time_(epoch_time) { CalcGpsWeekTime(); }
  /**
   * @fn GpsTime
   * @brief Constructor initialized with calender expression
   */
  GpsTime(const DateTime date_time) : date_time_(date_time), epoch_time_(EpochTime(date_time_)) { CalcGpsWeekTime(); }

  /**
   * @fn ~GpsTime
   * @brief Destructor
   */
  ~GpsTime() {}

  /**
   * @fn GetWeek
   * @return GPS week
   */
  inline size_t GetWeek() const { return week_; }
  /**
   * @fn GetTime
   * @return GPS time from week
   */
  inline double GetElapsedTimeFromWeek_s() const { return elapsed_time_from_week_s_; }
  /**
   * @fn GetEpochTime
   * @return GPS time in epoch time expression
   */
  inline EpochTime GetEpochTime() const { return epoch_time_; }
  /**
   * @fn GetDateTime
   * @return GPS time in date time expression
   */
  inline DateTime GetDateTime() const { return date_time_; }
  /**
   * @fn GetDateTimeAsUtc
   * @return DateTime as UTC including leap seconds
   */
  inline DateTime GetDateTimeAsUtc() const { return epoch_time_ - kLeapSecondAheadFromUtc_; }

 private:
  size_t week_;                      //!< GPS week (week = 0 at 6th Jan. 1980)
  double elapsed_time_from_week_s_;  //!< Elapsed time from the GPS week [s] [0,0, 604800.0)
  // Expressions
  DateTime date_time_;    //!< GPS time in date time expression
  EpochTime epoch_time_;  //!< GPS time in epoch time expression
  // Epoch of GPS time
  static const DateTime kEpochOfGpsTimeInDateTime_;    //!< GPS time epoch in date time expression
  static const EpochTime kEpochOfGpsTimeInEpochTime_;  //!< GPS time epoch in epoch time expression
  // Constants
  static const size_t kSecondsInWeek_ = 86400 * 7;  //!< Seconds in Week
  static const EpochTime kLeapSecondAheadFromUtc_;  //!< leap second ahead from UTC

  // Functions
  /**
   * @fn CalcGpsWeekTime
   * @brief Calculate GPS Time from epoch time
   */
  void CalcGpsWeekTime();
  /**
   * @fn CalcEpochTime
   * @brief Calculate Epoch time from GPS time
   */
  void CalcEpochTime();
};

}  // namespace s2e::time_system

#endif  // S2E_LIBRARY_TIME_SYSTEM_GPS_TIME_HPP_
