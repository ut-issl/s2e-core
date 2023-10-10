/**
 * @file initialize_node.cpp
 * @brief Initialize function for node
 */

#include "initialize_node.hpp"

#include <cassert>
#include <iostream>
#include <library/initialize/initialize_file_access.hpp>
#include <string>
#include <typeinfo>
#include <vector>

/* Import node properties by reading CSV File (node.csv)

[File Formats of node.csv]
column 1: Node_id(int)
column 2: Node_label(string)
column 3: Node_type(int, 0: diffusive, 1: boundary), Arithmetic node to be implemented as future work
column 4: Heater No
column 5: capacity
column 6: alpha
column 7: area
column 8,9,10: normal vector of surface(body frame)
column 11: initial temperature(K)

First row is for Header, data begins from the second row
Ex.
Node_id,Node_label,node_type,heater_id,capacity,alpha,area,normal_v_b_x,normal_v_b_y,normal_v_b_z,initial_temperature
0,BUS,0,1,880,0.2,0.06,1,0,0,300
1,SAP,0,0,100,0,0.02,0,0,1,250
2,SPACE,1,0,0,0,0,0,0,0,2.73

Be sure to include at least one boundary node to avoid divergence
*/

Node InitNode(const std::vector<std::string>& node_str) {
  using std::stod;
  using std::stoi;

  size_t node_str_size_defined = 11;                 // Correct size of node_str
  assert(node_str.size() == node_str_size_defined);  // Check if size of node_str is correct

  size_t node_id = 0;               // node number
  std::string node_label = "temp";  // node name
  size_t node_type_int = 0;         // node type
  size_t heater_id = 0;             // heater node index
  double temperature_K = 0;         // [K]
  double capacity_J_K = 0;          // [J/K]
  double alpha = 0;                 // []
  double area_m2 = 0;               // [m^2]

  // Index to read from node_str for each parameter
  size_t index_node_id = 0;
  size_t index_node_label = 1;
  size_t index_node_type = 2;
  size_t index_heater_id = 3;
  size_t index_capacity = 4;
  size_t index_alpha = 5;
  size_t index_area = 6;
  size_t index_normal_v_b_head = 7;
  size_t index_temperature = 10;

  node_id = stoi(node_str[index_node_id]);
  node_label = node_str[index_node_label];
  node_type_int = stoi(node_str[index_node_type]);
  heater_id = stoi(node_str[index_heater_id]);
  capacity_J_K = stod(node_str[index_capacity]);
  alpha = stod(node_str[index_alpha]);
  area_m2 = stod(node_str[index_area]);
  libra::Vector<3> normal_v_b;
  for (size_t i = 0; i < 3; i++) {
    normal_v_b[i] = stod(node_str[index_normal_v_b_head + i]);
  }

  // Normalize Norm Vector (Except for Boundary and Arithmetic Nodes)
  if (node_type_int == 0) {
    double norm = normal_v_b.CalcNorm();
    for (size_t i = 0; i < 3; i++) {
      normal_v_b[i] = normal_v_b[i] / norm;
    }
  }

  temperature_K = stod(node_str[index_temperature]);

  NodeType node_type = NodeType::kDiffusive;
  if (node_type_int == 1) {
    node_type = NodeType::kBoundary;
  } else if (node_type_int == 2) {
    node_type = NodeType::kArithmetic;
  }
  Node node(node_id, node_label, node_type, heater_id, temperature_K, capacity_J_K, alpha, area_m2, normal_v_b);
  return node;
}
