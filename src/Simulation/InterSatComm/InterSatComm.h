/**
 * @file InterSatComm.h
 * @brief Base class of inter satellite communication
 */

#pragma once

#include "../SimulationConfig.h"

/**
 * @class InterSatComm
 * @brief Base class of inter satellite communication
 */
class InterSatComm {
 public:
  /**
   * @fn InterSatComm
   * @brief Constructor
   */
  InterSatComm(const SimulationConfig* sim_config);
  /**
   * @fn ~InterSatComm
   * @brief Destructor
   */
  ~InterSatComm();

 private:
};