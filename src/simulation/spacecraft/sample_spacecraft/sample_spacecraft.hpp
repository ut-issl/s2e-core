/**
 * @file sample_spacecraft.hpp
 * @brief An example of user side spacecraft class
 */

#ifndef S2E_SIMULATION_SPACECRAFT_SAMPLE_SPACECRAFT_SAMPLE_SPACECRAFT_HPP_
#define S2E_SIMULATION_SPACECRAFT_SAMPLE_SPACECRAFT_SAMPLE_SPACECRAFT_HPP_

#include "../spacecraft.hpp"
#include "sample_components.hpp"

class SampleComponents;

/**
 * @class SampleSpacecraft
 * @brief An example of user side spacecraft class
 */
class SampleSpacecraft : public Spacecraft {
 public:
  /**
   * @fn SampleSpacecraft
   * @brief Constructor
   */
  SampleSpacecraft(const SimulationConfiguration* simulation_configuration, const GlobalEnvironment* global_environment,
                   const unsigned int spacecraft_id);

  /**
   * @fn GetInstalledComponents
   * @brief Get components installed on the spacecraft
   */
  inline const SampleComponents& GetInstalledComponents() const { return *sample_components_; }

 private:
  SampleComponents* sample_components_;
};

#endif  // S2E_SIMULATION_SPACECRAFT_SAMPLE_SPACECRAFT_SAMPLE_SPACECRAFT_HPP_
