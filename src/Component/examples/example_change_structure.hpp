/**
 * @file example_change_structure.hpp
 * @brief Class to show an example to change satellite structure information
 */

#ifndef CHANGE_STRUCTURE_H_
#define CHANGE_STRUCTURE_H_

#include <Interface/LogOutput/ILoggable.h>
#include <Simulation/Spacecraft/Structure/Structure.h>

#include "../Abstract/ComponentBase.h"

/**
 * @class ChangeStructure
 * @brief Class to show an example to change satellite structure information
 */
class ChangeStructure : public ComponentBase, public ILoggable {
 public:
  /**
   * @fn ChangeStructure
   * @brief Constructor with power port
   * @param [in] clock_gen: Clock generator
   * @param [in] structure: Structure information
   */
  ChangeStructure(ClockGenerator* clock_gen, Structure* structure);
  /**
   * @fn ~ChangeStructure
   * @brief Destructor
   */
  ~ChangeStructure();

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

#endif
