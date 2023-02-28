/**
 * @file simulation_object.cpp
 * @brief Class to manage randomization of variables for Monte-Carlo simulation
 */

#include "simulation_object.hpp"

std::map<std::string, SimulationObject*> SimulationObject::ojbect_list_;

SimulationObject::SimulationObject(std::string name) : name_(name) {
  // Check the name is already registered in so_list
  std::map<std::string, SimulationObject*>::iterator itr = SimulationObject::ojbect_list_.find(name);

  if (itr == SimulationObject::ojbect_list_.end()) {
    // Register itself in so_list if it is not registered yet
    SimulationObject::ojbect_list_[name] = this;
  } else {
    // If it is already registered in so_list, throw error. It should be deleted in the finalize phase of previous case.
    // Or there is a possibility that the same name SimulationObjects are registered in the list.
    throw "SimulationObject already defined. You may have forgotten to delete me in the previous simulation case, or defined two SimulationObjects with the same name.";
  }
}

SimulationObject::~SimulationObject() {
  // Remove itself from so_list
  SimulationObject::ojbect_list_.erase(name_);
}

void SimulationObject::SetAllParameters(const MonteCarloSimulationExecutor& monte_carlo_simulator) {
  for (auto so : SimulationObject::ojbect_list_) {
    so.second->SetParameters(monte_carlo_simulator);
  }
}

void SimulationObject::GetInitParameterDouble(const MonteCarloSimulationExecutor& monte_carlo_simulator, std::string ip_name,
                                              double& destination) const {
  monte_carlo_simulator.GetInitParameterDouble(name_, ip_name, destination);
}

void SimulationObject::GetInitParameterQuaternion(const MonteCarloSimulationExecutor& monte_carlo_simulator, std::string ip_name,
                                                  libra::Quaternion& destination) const {
  monte_carlo_simulator.GetInitParameterQuaternion(name_, ip_name, destination);
}
