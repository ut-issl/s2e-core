/**
 * @file date_time_format.hpp
 * @brief Class to handle Gregorian date and time format
 */

#ifndef S2E_LIBRARY_TIME_SYSTEM_DATE_TIME_FORMAT_HPP_
#define S2E_LIBRARY_TIME_SYSTEM_DATE_TIME_FORMAT_HPP_

#include <string>

/**
 *@class DateTime
 * @brief Class to handle Gregorian date and time format
 */
class DateTime {
 public:
  /**
   * @fn DateTime
   * @brief Constructor initialized with full member information
   * @note This format do not define leap seconds and time differences. Users of the time expression should define them when using.
   */
  DateTime(const size_t year, const size_t month, const size_t day, const size_t hour, const size_t minute, const double second)
      : year_(year), month_(month), day_(day), hour_(hour), minute_(minute), second_(second) {}
  /**
   * @fn DateTime
   * @brief Constructor initialized with string expression as YYYY/MM/DD hh:mm:ss.s
   * @note TODO: Support other format like dd.mm.yyyy
   */
  DateTime(const std::string date_time = "0000/01/01 00:00:00.0") {
    sscanf(date_time.c_str(), "%zu/%zu/%zu %zu:%zu:%lf", &year_, &month_, &day_, &hour_, &minute_, &second_);
  }

  // Getters
  inline size_t GetYear() const { return year_; }
  inline size_t GetMonth() const { return month_; }
  inline size_t GetDay() const { return day_; }
  inline size_t GetHour() const { return hour_; }
  inline size_t GetMinute() const { return minute_; }
  inline double GetSecond() const { return second_; }
  /**
   * @fn GetAsString
   * @brief Return with string expression as YYYY/MM/DD hh:mm:ss.s
   * @note TODO: Support other format like dd.mm.yyyy
   */
  std::string GetAsString() const;

 private:
  size_t year_;    //!< Year [0, -]
  size_t month_;   //!< Month [1, 12]
  size_t day_;     //!< Day [1 to 31]
  size_t hour_;    //!< Hour [0 to 23]
  size_t minute_;  //!< Minute [0 to 59]
  double second_;  //!< Second [0.0, 60.0)
};

#endif  // S2E_LIBRARY_TIME_SYSTEM_DATE_TIME_FORMAT_HPP_
