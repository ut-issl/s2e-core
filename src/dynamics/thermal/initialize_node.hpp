/**
 * @file initialize_node.hpp
 * @brief Initialize function for node
 */

#ifndef S2E_DYNAMICS_THERMAL_INITIALIZE_NODE_HPP_
#define S2E_DYNAMICS_THERMAL_INITIALIZE_NODE_HPP_

#include "node.hpp"

/**
 * @fn InitNode
 * @brief Initialize Node object from csv file
 * @param[in] node_str: str read from csv file
 * @return Node
 */
Node InitNode(const std::vector<std::string>& node_str);

#endif  // S2E_DYNAMICS_THERMAL_INITIALIZE_NODE_HPP_
