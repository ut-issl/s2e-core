/**
 * @file loggable.hpp
 * @brief Abstract class to manage logging
 */

#ifndef S2E_LIBRARY_LOGGER_LOGGABLE_HPP_
#define S2E_LIBRARY_LOGGER_LOGGABLE_HPP_

#include <string>

#include "log_utility.hpp"  // This is not necessary but include here for convenience

/**
 * @class ILoggable
 * @brief Abstract class to manage logging
 * @note We wan to make this as an interface class, but to handle enable flag, we made this as abstract class
 */
class ILoggable {
 public:
  /**
   * @fn GetLogHeader
   * @brief Get headers to write in CSV output file
   * @return The headers
   */
  virtual std::string GetLogHeader() const = 0;

  /**
   * @fn GetLogValue
   * @brief Get values to write in CSV output file
   * @return The output values
   */
  virtual std::string GetLogValue() const = 0;

  bool is_log_enabled_ = true;  //!< Log enable flag
};

#endif  // S2E_LIBRARY_LOGGER_LOGGABLE_HPP_
