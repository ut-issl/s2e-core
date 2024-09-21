/**
 * @file simulation_object.cpp
 * @brief Class to manage randomization of variables for Monte-Carlo simulation
 */

#include "simulation_object.hpp"

std::map<std::string, SimulationObject*> SimulationObject::object_list_;

SimulationObject::SimulationObject(std::string name) : name_(name) {
  // Check the name is already registered in so_list
  std::map<std::string, SimulationObject*>::iterator itr = SimulationObject::object_list_.find(name);

  if (itr == SimulationObject::object_list_.end()) {
    // Register itself in so_list if it is not registered yet
    SimulationObject::object_list_[name] = this;
  } else {
    // If it is already registered in so_list, throw error. It should be deleted in the finalize phase of previous case.
    // Or there is a possibility that the same name SimulationObjects are registered in the list.
    throw "SimulationObject already defined. You may have forgotten to delete me in the previous simulation case, or defined two SimulationObjects with the same name.";
  }
}

SimulationObject::~SimulationObject() {
  // Remove itself from so_list
  SimulationObject::object_list_.erase(name_);
}

void SimulationObject::SetAllParameters(const MonteCarloSimulationExecutor& monte_carlo_simulator) {
  for (auto so : SimulationObject::object_list_) {
    so.second->SetParameters(monte_carlo_simulator);
  }
}

void SimulationObject::GetInitializedMonteCarloParameterDouble(const MonteCarloSimulationExecutor& monte_carlo_simulator,
                                                               std::string init_monte_carlo_parameter_name, double& destination) const {
  monte_carlo_simulator.GetInitializedMonteCarloParameterDouble(name_, init_monte_carlo_parameter_name, destination);
}

void SimulationObject::GetInitializedMonteCarloParameterQuaternion(const MonteCarloSimulationExecutor& monte_carlo_simulator,
                                                                   std::string init_monte_carlo_parameter_name, math::Quaternion& destination) const {
  monte_carlo_simulator.GetInitializedMonteCarloParameterQuaternion(name_, init_monte_carlo_parameter_name, destination);
}
