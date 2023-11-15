/**
 * @file epoch_time.hpp
 * @brief A class to handle time like UNIX time with fractions
 */

#ifndef S2E_LIBRARY_TIME_SYSTEM_EPOCH_TIME_HPP_
#define S2E_LIBRARY_TIME_SYSTEM_EPOCH_TIME_HPP_

#include <cstdint>

#include "date_time_format.hpp"

/**
 * @class EpochTime
 * @brief A class to handle time like UNIX time with fractions.
 * @note This class doesn't care leap seconds.
 */
class EpochTime {
 public:
  /**
   * @fn EpochTime
   * @brief Constructor initialized with week and second
   */
  EpochTime(const uint64_t time_s = 0, const double fraction_s = 0.0) : time_s_(time_s), fraction_s_(fraction_s) {}
  /**
   * @fn EpochTime
   * @brief Constructor initialized with date time expression
   */
  EpochTime(const DateTime date_time);
  /**
   * @fn ~EpochTime
   * @brief Destructor
   */
  ~EpochTime() {}

  // Getter
  /**
   * @fn GetTime_s
   * @return time [s]
   */
  inline uint64_t GetTime_s() const { return time_s_; }
  /**
   * @fn GetFraction_s
   * @return fraction time [s]
   */
  inline double GetFraction_s() const { return fraction_s_; }
  /**
   * @fn GetTimeWithFraction_s
   * @return time + fraction [s]
   */
  inline double GetTimeWithFraction_s() const { return (double)(time_s_) + fraction_s_; }

  // Operator
  bool operator==(const EpochTime& target) const;
  bool operator!=(const EpochTime& target) const { return !(*this == target); }
  bool operator>(const EpochTime& right_side) const;
  bool operator<=(const EpochTime& right_side) const { return !(*this > right_side); }
  bool operator<(const EpochTime& right_side) const;
  bool operator>=(const EpochTime& right_side) const { return !(*this < right_side); }
  EpochTime operator+(const EpochTime& right_side) const;
  EpochTime operator-(const EpochTime& right_side) const;

 private:
  uint64_t time_s_;    //!< Number of seconds without leap seconds since 00:00:00 Jan 1 1970 UTC.
  double fraction_s_;  //!< Fraction of second under 1 sec [0, 1)
};

#endif  // S2E_LIBRARY_TIME_SYSTEM_EPOCH_TIME_HPP_
