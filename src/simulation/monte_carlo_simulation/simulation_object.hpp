/**
 * @file simulation_object.h
 * @brief Class to manage randomization of variables for Monte-Carlo simulation
 */

#ifndef S2E_SIMULATION_MONTE_CARLO_SIMULATION_SIMULATION_OBJECT_HPP_
#define S2E_SIMULATION_MONTE_CARLO_SIMULATION_SIMULATION_OBJECT_HPP_

#include <functional>
#include <math_physics/math/quaternion.hpp>
#include <math_physics/math/vector.hpp>
#include <map>
#include <string>
#include <vector>

#include "initialize_monte_carlo_parameters.hpp"
#include "monte_carlo_simulation_executor.hpp"

/**
 * @class SimulationObject
 * @brief Class to manage randomization of variables for Monte-Carlo simulation
 */
class SimulationObject {
 public:
  /**
   * @fn SimulationObject
   * @brief Constructor
   */
  explicit SimulationObject(std::string name);

  /**
   * @fn ~SimulationObject
   * @brief Destructor
   */
  virtual ~SimulationObject();

  /**
   * @fn GetInitializedMonteCarloParameterVector
   * @brief Get randomized vector value and store it in destination
   */
  template <size_t NumElement>
  void GetInitializedMonteCarloParameterVector(const MonteCarloSimulationExecutor& monte_carlo_simulator, std::string init_monte_carlo_parameter_name,
                                               libra::Vector<NumElement>& destination) const;

  /**
   * @fn GetInitializedMonteCarloParameterDouble
   * @brief Get randomized value and store it in destination
   */
  void GetInitializedMonteCarloParameterDouble(const MonteCarloSimulationExecutor& monte_carlo_simulator, std::string init_monte_carlo_parameter_name,
                                               double& destination) const;

  /**
   * @fn GetInitializedMonteCarloParameterQuaternion
   * @brief Get randomized quaternion and store it in destination
   */
  void GetInitializedMonteCarloParameterQuaternion(const MonteCarloSimulationExecutor& monte_carlo_simulator,
                                                   std::string init_monte_carlo_parameter_name, libra::Quaternion& destination) const;

  /**
   * @fn SetParameters
   * @brief Virtual function to set the randomized results to target variables
   */
  virtual void SetParameters(const MonteCarloSimulationExecutor& monte_carlo_simulator) = 0;

  /**
   * @fn SetAllParameters
   * @brief Execute all SetParameter function for all SimulationObject instance
   */
  static void SetAllParameters(const MonteCarloSimulationExecutor& monte_carlo_simulator);

 private:
  std::string name_;  //!< Name to distinguish the target variable in initialize file for Monte-Carlo simulation
  static std::map<std::string, SimulationObject*> object_list_;  //!< list of objects with simulation parameters
};

/**
 * @fn GetInitializedMonteCarloParameterVector
 * @brief Return initialized parameters for vector
 */
template <size_t NumElement>
void SimulationObject::GetInitializedMonteCarloParameterVector(const MonteCarloSimulationExecutor& monte_carlo_simulator,
                                                               std::string init_monte_carlo_parameter_name,
                                                               libra::Vector<NumElement>& destination) const {
  monte_carlo_simulator.GetInitializedMonteCarloParameterVector(name_, init_monte_carlo_parameter_name, destination);
}

#endif  // S2E_SIMULATION_MONTE_CARLO_SIMULATION_SIMULATION_OBJECT_HPP_
