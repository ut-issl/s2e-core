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
 * @brief A class to define epoch time expression
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
   * @brief Constructor initialized with full member information
   */
  EpochTime(const DateTime date_time);
  /**
   * @fn ~EpochTime
   * @brief Destructor
   */
  ~EpochTime() {}

  // Getter
  inline uint64_t GetTime_s() const { return time_s_; }
  inline double GetFraction_s() const { return fraction_s_; }
  inline double GetTimeWithFraction_s() const { return (double)(time_s_) + fraction_s_; }

  // Operator
  bool operator==(const EpochTime& target) const {
    if (this->time_s_ != target.time_s_) return false;
    if (this->fraction_s_ != target.fraction_s_) return false;  // TODO: comparison of double
    return true;
  }
  bool operator!=(const EpochTime& target) const { return !(*this == target); }

  bool operator>(const EpochTime& right_side) const {
    if (this->time_s_ < right_side.time_s_) return false;
    if (this->time_s_ > right_side.time_s_) return true;
    if (this->fraction_s_ < right_side.fraction_s_) return false;
    return true;
  }
  bool operator<=(const EpochTime& right_side) const { return !(*this > right_side); }

  bool operator<(const EpochTime& right_side) const {
    if (this->time_s_ > right_side.time_s_) return false;
    if (this->time_s_ < right_side.time_s_) return true;
    if (this->fraction_s_ > right_side.fraction_s_) return false;
    return true;
  }
  bool operator>=(const EpochTime& right_side) const { return !(*this < right_side); }

 private:
  uint64_t time_s_;    //!< Number of seconds without leap seconds since 00:00:00 Jan 1 1970 UTC.
  double fraction_s_;  //!< Fraction of second under 1 sec
};

// Calculation
EpochTime TimeAdd(const EpochTime& left_side, const EpochTime& right_side);
EpochTime TimeSubtract(const EpochTime& left_side, const EpochTime& right_side);

#endif  // S2E_LIBRARY_TIME_SYSTEM_EPOCH_TIME_HPP_
