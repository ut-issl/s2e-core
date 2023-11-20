/**
 * @file gps_time.hpp
 * @brief A class to define GPS time expression
 */

#ifndef S2E_LIBRARY_TIME_SYSTEM_GPS_TIME_HPP_
#define S2E_LIBRARY_TIME_SYSTEM_GPS_TIME_HPP_

#include <cstddef>

#include "epoch_time.hpp"

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
  GpsTime(const size_t week = 0, const double elapsed_time_from_week_s = 0.0) : week_(week), elapsed_time_from_week_s_(elapsed_time_from_week_s) {}
  /**
   * @fn GpsTime
   * @brief Constructor initialized with epoch time expression
   */
  GpsTime(const EpochTime epoch_time) : epoch_time_(epoch_time) { CalcGpsWeekTime(); }

  ~GpsTime() {}

  /**
   * @fn GetWeek
   * @return GPS week
   */
  inline size_t GetWeek() const { return week_; }
  /**
   * @fn GetTime
   * @return GPS time
   */
  inline double GetElapsedTimeFromWeek_s() const { return elapsed_time_from_week_s_; }

 private:
  size_t week_;                      //!< GPS week (week = 0 at 6th Jan. 1980)
  double elapsed_time_from_week_s_;  //!< Elapsed time from the GPS week [s] [0,0, 604800.0)
  // Expressions
  EpochTime epoch_time_;  //!< GPS time in epoch time expression
  // Epoch of GPS time
  static const EpochTime epoch_of_gps_time_in_epoch_time_;  //!< GPS time epoch in epoch time expression
  // Constants
  static const size_t seconds_in_week_ = 86400 * 7;

  // Functions
  void CalcGpsWeekTime();
};

#endif  // S2E_LIBRARY_TIME_SYSTEM_GPS_TIME_HPP_
