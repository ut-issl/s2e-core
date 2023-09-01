/*
 * @file wings_command_sender_to_c2a.hpp
 * @brief A component to send command using WINGS operation file
 */

#ifndef S2E_COMPONENTS_REAL_COMMUNICATION_C2A_COMMAND_SENDER_HPP_
#define S2E_COMPONENTS_REAL_COMMUNICATION_C2A_COMMAND_SENDER_HPP_

#include <library/initialize/c2a_command_database.hpp>
#include <library/initialize/wings_operation_file.hpp>

#include "../../base/component.hpp"

/*
 * @class C2aCommandSender
 * @brief A component to send command using WINGS operation file
 */
class WingsCommandSenderToC2a : public Component {
 public:
  /**
   * @fn WingsCommandSenderToC2a
   * @brief Constructor
   * @param [in] 
   */
  WingsCommandSenderToC2a(int prescaler, ClockGenerator* clock_generator, const double step_width_s, const std::string command_database_file,
                          const std::string operation_file)
      : Component(prescaler, clock_generator),
        c2a_command_database_(command_database_file),
        wings_operation_file_(operation_file, c2a_command_database_),
        step_width_s_(step_width_s) {}

  /**
   * @fn ~WingsCommandSenderToC2a
   * @brief Destructor
   */
  ~WingsCommandSenderToC2a() {}

 protected:
  C2aCommandDatabase c2a_command_database_;  //!< Command database
  WingsOperationFile wings_operation_file_;  //!< WINGS operation file
  const double step_width_s_; //!< Step width to execute this component
  double wait_s_;

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to send command
   */
  void MainRoutine(const int time_count) override;
};

WingsCommandSenderToC2a InitWingsCommandSenderToC2a(ClockGenerator* clock_generator, const std::string initialize_file);

#endif  // S2E_COMPONENTS_REAL_COMMUNICATION_C2A_COMMAND_SENDER_HPP_
