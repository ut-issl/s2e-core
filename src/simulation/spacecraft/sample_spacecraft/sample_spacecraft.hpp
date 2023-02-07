/**
 * @file sample_spacecraft.hpp
 * @brief An example of user side spacecraft class
 */

#pragma once

#include "../spacecraft.hpp"
#include "sample_components.hpp"

class SampleComponents;

/**
 * @class SampleSat
 * @brief An example of user side spacecraft class
 */
class SampleSat : public Spacecraft {
 public:
  /**
   * @fn SampleSat
   * @brief Constructor
   */
  SampleSat(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id);

  /**
   * @fn GetInstalledComponents
   * @brief Get components installed on the spacecraft
   */
  inline const SampleComponents& GetInstalledComponents() const { return *sample_components_; }

 private:
  SampleComponents* sample_components_;
};
