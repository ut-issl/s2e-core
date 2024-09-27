/**
 * @file sample_spacecraft.hpp
 * @brief An example of user side spacecraft class
 */

#ifndef S2E_SIMULATION_SAMPLE_SPACECRAFT_SAMPLE_SPACECRAFT_HPP_
#define S2E_SIMULATION_SAMPLE_SPACECRAFT_SAMPLE_SPACECRAFT_HPP_

#include <src/simulation/spacecraft/spacecraft.hpp>

#include "sample_components.hpp"

class SampleComponents;

namespace s2e::sample {

/**
 * @class SampleSpacecraft
 * @brief An example of user side spacecraft class
 */
class SampleSpacecraft : public simulation::Spacecraft {
 public:
  /**
   * @fn SampleSpacecraft
   * @brief Constructor
   */
  SampleSpacecraft(const simulation::SimulationConfiguration* simulation_configuration, const environment::GlobalEnvironment* global_environment,
                   const unsigned int spacecraft_id);

  /**
   * @fn GetInstalledComponents
   * @brief Get components installed on the spacecraft
   */
  inline const SampleComponents& GetInstalledComponents() const { return *sample_components_; }

 private:
  SampleComponents* sample_components_;
};

} // namespace s2e::sample

#endif  // S2E_SIMULATION_SAMPLE_SPACECRAFT_SAMPLE_SPACECRAFT_HPP_
