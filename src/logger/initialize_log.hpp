/**
 * @file initialize_log.hpp
 * @brief Initialize function for Log output
 */

#ifndef S2E_LIBRARY_LOGGER_INITIALIZE_LOG_HPP_
#define S2E_LIBRARY_LOGGER_INITIALIZE_LOG_HPP_

#include <logger/logger.hpp>

namespace s2e::logger {

/**
 * @fn InitLog
 * @brief Initialize normal logger (default.csv)
 * @param [in] file_name: File name of the log file
 */
Logger* InitLog(std::string file_name);

/**
 * @fn InitMonteCarloLog
 * @brief Initialize logger for Monte-Carlo simulation (mont.csv)
 * @param [in] file_name: File name of the log file
 * @param [in] enable: Enable flag for logging
 */
Logger* InitMonteCarloLog(std::string file_name, bool enable);

} // namespace s2e::logger

#endif  // S2E_LIBRARY_LOGGER_INITIALIZE_LOG_HPP_
