/**
 * @file example_change_structure.hpp
 * @brief Class to show an example to change satellite structure information
 */

#pragma once

#include <Interface/LogOutput/ILoggable.h>
#include <simulation/spacecraft/structure/Structure.h>

#include "../Abstract/ComponentBase.h"

/**
 * @class ExampleChangeStructure
 * @brief Class to show an example to change satellite structure information
 */
class ExampleChangeStructure : public ComponentBase, public ILoggable {
 public:
  /**
   * @fn ExampleChangeStructure
   * @brief Constructor with power port
   * @param [in] clock_gen: Clock generator
   * @param [in] structure: Structure information
   */
  ExampleChangeStructure(ClockGenerator* clock_gen, Structure* structure);
  /**
   * @fn ~ChangeStructure
   * @brief Destructor
   */
  ~ExampleChangeStructure();

  // Override functions for ComponentBase
  /**
   * @fn MainRoutine
   * @brief Main routine for sensor observation
   */
  void MainRoutine(int count) override;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

 protected:
  Structure* structure_;  //!< Structure information
};
