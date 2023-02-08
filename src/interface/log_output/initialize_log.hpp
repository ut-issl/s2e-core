/**
 * @file initialize_log.hpp
 * @brief Initialize function for Log output
 */

#ifndef S2E_INTERFACE_LOG_OUTPUT_INITIALIZE_LOG_H_
#define S2E_INTERFACE_LOG_OUTPUT_INITIALIZE_LOG_H_

#include <interface/log_output/Logger.h>

/**
 * @fn InitLog
 * @brief Initialize normal logger (default.csv)
 * @param [in] file_name: File name of the log file
 */
Logger* InitLog(std::string file_name);

/**
 * @fn InitLogMC
 * @brief Initialize logger for Monte-Carlo simulation (mont.csv)
 * @param [in] file_name: File name of the log file
 * @param [in] enable: Enable flag for logging
 */
Logger* InitLogMC(std::string file_name, bool enable);

#endif  // S2E_INTERFACE_LOG_OUTPUT_INITIALIZE_LOG_H_
