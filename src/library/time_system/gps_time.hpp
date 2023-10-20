/**
 * @file gps_time.hpp
 * @brief A class to define GPS time expression
 */

#ifndef S2E_LIBRARY_TIME_SYSTEM_GPS_TIME_HPP_
#define S2E_LIBRARY_TIME_SYSTEM_GPS_TIME_HPP_

#include <cstddef>

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
  GpsTime(const size_t week = 0, const double time_of_week = 0.0) : week_(week), time_of_week_(time_of_week) {}

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
  inline double GetTimeOfWeek() const { return time_of_week_; }

 private:
  size_t week_;          //!< GPS week (week = 0 at 6th Jan. 1980)
  double time_of_week_;  //!< Time of week [0,0, 604800.0)
};

#endif  // S2E_LIBRARY_TIME_SYSTEM_GPS_TIME_HPP_