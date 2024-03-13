/**
 * @file sample_spacecraft.cpp
 * @brief An example of user side spacecraft class
 */

#include "sample_spacecraft.hpp"

#include <environment/global/clock_generator.hpp>
#include <math_physics/randomization/normal_randomization.hpp>

#include "sample_components.hpp"

SampleSpacecraft::SampleSpacecraft(const SimulationConfiguration* simulation_configuration, const GlobalEnvironment* global_environment,
                                   const unsigned int spacecraft_id)
    : Spacecraft(simulation_configuration, global_environment, spacecraft_id) {
  sample_components_ =
      new SampleComponents(dynamics_, structure_, local_environment_, global_environment, simulation_configuration, &clock_generator_, spacecraft_id);
  components_ = sample_components_;
}
