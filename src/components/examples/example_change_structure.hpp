/**
 * @file example_change_structure.hpp
 * @brief Class to show an example to change satellite structure information
 */

#ifndef S2E_COMPONENTS_EXAMPLES_EXAMPLE_CHANGE_STRUCTURE_HPP_
#define S2E_COMPONENTS_EXAMPLES_EXAMPLE_CHANGE_STRUCTURE_HPP_

#include <logger/loggable.hpp>
#include <simulation/spacecraft/structure/structure.hpp>

#include "../base/component.hpp"

namespace s2e::components {

/**
 * @class ExampleChangeStructure
 * @brief Class to show an example to change satellite structure information
 */
class ExampleChangeStructure : public Component, public logger::ILoggable {
 public:
  /**
   * @fn ExampleChangeStructure
   * @brief Constructor with power port
   * @param [in] clock_generator: Clock generator
   * @param [in] structure: Structure information
   */
  ExampleChangeStructure(environment::ClockGenerator* clock_generator, spacecraft::Structure* structure);
  /**
   * @fn ~ChangeStructure
   * @brief Destructor
   */
  ~ExampleChangeStructure();

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(const int time_count) override;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const override;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const override;

 protected:
  spacecraft::Structure* structure_;  //!< Structure information
};

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_EXAMPLES_EXAMPLE_CHANGE_STRUCTURE_HPP_
