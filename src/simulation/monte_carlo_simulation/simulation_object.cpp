/**
 * @file simulation_object.cpp
 * @brief Class to manage randomization of variables for Monte-Carlo simulation
 */

#include "simulation_object.hpp"

std::map<std::string, SimulationObject*> SimulationObject::so_list_;

SimulationObject::SimulationObject(std::string name) : name_(name) {
  // Check the name is already registered in so_list
  std::map<std::string, SimulationObject*>::iterator itr = SimulationObject::so_list_.find(name);

  if (itr == SimulationObject::so_list_.end()) {
    // Register itself in so_list if it is not registered yet
    SimulationObject::so_list_[name] = this;
  } else {
    // If it is already registered in so_list, throw error. It should be deleted in the finalize phase of previous case.
    // Or there is a possibility that the same name SimulationObjects are registered in the list.
    throw "SimulationObject already defined. You may have forgotten to delete me in the previous simulation case, or defined two SimulationObjects with the same name.";
  }
}

SimulationObject::~SimulationObject() {
  // Remove itself from so_list
  SimulationObject::so_list_.erase(name_);
}

void SimulationObject::SetAllParameters(const MCSimExecutor& mc_sim) {
  for (auto so : SimulationObject::so_list_) {
    so.second->SetParameters(mc_sim);
  }
}

void SimulationObject::GetInitParameterDouble(const MCSimExecutor& mc_sim, std::string ip_name, double& dst) const {
  mc_sim.GetInitParameterDouble(name_, ip_name, dst);
}

void SimulationObject::GetInitParameterQuaternion(const MCSimExecutor& mc_sim, std::string ip_name, Quaternion& dst_quat) const {
  mc_sim.GetInitParameterQuaternion(name_, ip_name, dst_quat);
}
