/**
 * @file initialize_node.cpp
 * @brief Initialize function for node
 */

#include "initialize_node.hpp"

#include <iostream>
#include <library/initialize/initialize_file_access.hpp>
#include <string>
#include <typeinfo>
#include <vector>

/* Import node properties by reading CSV File (Node.csv)

[File Formats of Node.csv]
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
Node_id,Node_label,node_type,heater_node_id,capacity,alpha,area,normal_v_b_x,normal_v_b_y,normal_v_b_z,initial_temperature
0,BUS,0,1,880,0.2,0.06,1,0,0,300
1,SAP,0,0,100,0,0.02,0,0,1,250
2,SPACE,1,0,0,0,0,0,0,0,2.73

Be sure to include at least one boundary node to avoid divergence
*/

Node InitNode(const std::vector<std::string>& node_str) {
  using std::stod;
  using std::stoi;

  int node_id = 0;                  // node number
  std::string node_label = "temp";  // node name
  int node_type = 0;                // node type
  int heater_node_id = 0;           // heater node index
  double temperature = 0;           // [K]
  double capacity = 0;              // [J/K]
  double alpha = 0;                 //[m^2]
  double area = 0;

  node_id = stoi(node_str[0]);         // column 1
  node_label = node_str[1];            // column 2
  node_type = stoi(node_str[2]);       // column 3
  heater_node_id = stoi(node_str[3]);  // column 4
  capacity = stod(node_str[4]);        // column 5
  alpha = stod(node_str[5]);           // column 6
  area = stod(node_str[6]);            // column 7
  libra::Vector<3> normal_v_b;         // column 8-10
  for (int i = 0; i < 3; i++) {
    normal_v_b[i] = stod(node_str[7 + i]);
  }                                  // body frame
  temperature = stod(node_str[10]);  // column 11

  Node node(node_id, node_label, node_type, heater_node_id, temperature, capacity, alpha, area, normal_v_b);
  return node;
}
