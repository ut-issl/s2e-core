/**
 * @file node.hpp
 * @brief thermal calculation node
 */

#ifndef S2E_DYNAMICS_THERMAL_NODE_HPP_
#define S2E_DYNAMICS_THERMAL_NODE_HPP_

#include <environment/global/physical_constants.hpp>
#include <logger/logger.hpp>
#include <string>
#include <vector>

/**
 * @enum NodeType
 * @brief Type of node
 */
enum class NodeType {
  //! Diffusive Node: Calculate temperature based on heat flow
  kDiffusive,
  //! Boundary Node: Fixed temperature
  kBoundary,
  //! Arithmetic Node: Node without heat capacity, calculate temperature after diffusive nodes
  kArithmetic
};

/**
 * @class Node
 * @brief Class for managing each node of model
 */
class Node {
 public:
  /**
   * @fn Node
   * @brief Construct a new Node object
   *
   * @param[in] node_id: ID of node
   * @param[in] node_name: Name of node (used for logging/output)
   * @param[in] node_type: NodeType
   * @param[in] heater_id: ID of heater attached to this node (0 if no heater)
   * @param[in] temperature_ini_K: Initial temperature of node [K]
   * @param[in] capacity_J_K: Heat capacity of node [J/K]
   * @param[in] alpha: Solar absorptivity of face with possibility of solar incidence
   * @param[in] area_m2: Area of face with possibility of solar incidence [m^2]
   * @param[in] normal_vector_b: Normal vector of face with possibility of solar incidence (Body frame)
   */
  Node(const size_t node_id, const std::string node_name, const NodeType node_type, const size_t heater_id, const double temperature_ini_K,
       const double capacity_J_K, const double alpha, const double area_m2, libra::Vector<3> normal_vector_b);
  /**
   * @fn ~Node
   * @brief Destroy the Node object
   */
  virtual ~Node();
  /**
   * @fn CalcSolarRadiation_W
   * @brief Calculate solar radiation [W] from sun direction, alpha, area, and normal vector
   *
   * @param sun_direction_b: Sun direction in body frame
   * @return double: Solar Radiation [W]
   */
  double CalcSolarRadiation_W(libra::Vector<3> sun_direction_b, double solar_flux_W_m2);

  // Getter
  /**
   * @fn GetNodeId
   * @brief Return Node Id
   * @return size_t: Node ID
   */
  inline size_t GetNodeId(void) const { return node_id_; }
  /**
   * @fn GetNodeName
   * @brief Return Node Name
   * @return std::string: Node name
   */
  inline std::string GetNodeName(void) const { return node_name_; }
  /**
   * @fn GetHeaterID
   * @brief Return Heater Id
   * @return int: Heater ID
   */
  inline size_t GetHeaterId(void) const { return heater_id_; }
  /**
   * @fn GetTemperature_K
   * @brief Get temperature of node in Kelvin
   * @return double: temperature [K]
   */
  inline double GetTemperature_K(void) const { return temperature_K_; }
  /**
   * @fn GetTemperature_degC
   * @brief Get temperature of node in degC
   * @return double: temperature [degC]
   */
  inline double GetTemperature_degC(void) const { return K2degC(temperature_K_); }
  /**
   * @fn GetCapacity_J_K
   * @brief Return heat capacity of node [J/K]
   * @return double: heat capacity [J/K]
   */
  inline double GetCapacity_J_K(void) const { return capacity_J_K_; }
  /**
   * @fn GetSolarRadiation_W
   * @brief Return Solar Radiation [W]
   * @return double: Solar Radiation [W]
   */
  inline double GetSolarRadiation_W(void) const { return solar_radiation_W_; }
  /**
   * @fn GetNodeType
   * @brief Return Node Type
   * @return NodeType
   */
  inline NodeType GetNodeType(void) const { return node_type_; }

  // Setter
  /**
   * @fn SetTemperature_K
   * @brief Set the temperature of node in Kelvin
   *
   * @param temperature_K
   */
  inline void SetTemperature_K(double temperature_K) { temperature_K_ = temperature_K; }

  // for debug
  /**
   * @fn PrintParam
   * @brief Print parameters of node in debug output
   */
  void PrintParam(void);

 protected:
  size_t node_id_;
  std::string node_name_;
  size_t heater_id_;
  double temperature_K_;
  double capacity_J_K_;
  double alpha_;
  double area_m2_;
  double solar_radiation_W_;
  NodeType node_type_;
  libra::Vector<3> normal_vector_b_;

  /**
   * @fn AssertNodeParams
   * @brief Check if Node Parameters are Correct
   */
  void AssertNodeParams(void);
};

/**
 * @fn InitNode
 * @brief Initialize Node object from csv file
 * @param[in] node_str: str read from csv file
 * @return Node
 */
Node InitNode(const std::vector<std::string>& node_str);

#endif  // S2E_DYNAMICS_THERMAL_NODE_HPP_
