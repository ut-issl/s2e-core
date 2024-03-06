/*
 * @file wings_command_sender_to_c2a.hpp
 * @brief A component to send command using WINGS operation file
 */

#ifndef S2E_COMPONENTS_REAL_COMMUNICATION_C2A_COMMAND_SENDER_HPP_
#define S2E_COMPONENTS_REAL_COMMUNICATION_C2A_COMMAND_SENDER_HPP_

#include <setting_file_reader/c2a_command_database.hpp>
#include <setting_file_reader/wings_operation_file.hpp>

#include "../../base/component.hpp"

/*
 * @class C2aCommandSender
 * @brief A component to send C2A command using WINGS operation file
 */
class WingsCommandSenderToC2a : public Component {
 public:
  /**
   * @fn WingsCommandSenderToC2a
   * @brief Constructor
   * @param [in]
   */
  WingsCommandSenderToC2a(int prescaler, ClockGenerator* clock_generator, const double step_width_s, const std::string command_database_file,
                          const std::string operation_file, const bool is_enabled)
      : Component(prescaler, clock_generator),
        c2a_command_database_(command_database_file),
        wings_operation_file_(operation_file),
        is_enabled_(is_enabled),
        step_width_s_(step_width_s) {}

  /**
   * @fn ~WingsCommandSenderToC2a
   * @brief Destructor
   */
  ~WingsCommandSenderToC2a() {}

 protected:
  C2aCommandDatabase c2a_command_database_;  //!< Command database
  WingsOperationFile wings_operation_file_;  //!< WINGS operation file
  bool is_enabled_;                          //!< Enable flag
  const double step_width_s_;                //!< Step width to execute this component [s]
  double wait_s_ = 0.0;                      //!< Wait counter [s]
  bool is_end_of_line_ = false;              //!< Flag to detect end of line

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to send command
   */
  void MainRoutine(const int time_count) override;

  /**
   * @fn ExecuteCommandLine
   * @brief Execute command line
   * @param[in] line: Executed line
   */
  void ExecuteCommandLine(const std::string line);

  /**
   * @fn AnalyzeC2aCommand
   * @brief Analyze C2A command Line
   * @param[in] tokens: Command line after space separation
   */
  void AnalyzeC2aCommand(const std::vector<std::string> tokens);
};

/**
 * @fn InitWingsCommandSenderToC2a
 * @brief Initialize WingsCommandSenderToC2a
 * @param[in] clock_generator: Clock generator
 * @param[in] compo_update_step_s: Component update step time [s]
 * @param[in] initialize_file: Initialize file name
 */
WingsCommandSenderToC2a InitWingsCommandSenderToC2a(ClockGenerator* clock_generator, const double compo_update_step_s,
                                                    const std::string initialize_file);

#endif  // S2E_COMPONENTS_REAL_COMMUNICATION_C2A_COMMAND_SENDER_HPP_
