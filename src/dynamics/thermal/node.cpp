/**
 * @file node.cpp
 * @brief thermal calculation node
 */

#include "node.hpp"

#include <cassert>
#include <cmath>
#include <environment/global/physical_constants.hpp>

using namespace std;
using namespace libra;

Node::Node(const size_t node_id, const string node_name, const NodeType node_type, const size_t heater_id, const double temperature_ini_K,
           const double capacity_J_K, const double alpha, const double area_m2, math::Vector<3> normal_vector_b)
    : node_id_(node_id),
      node_name_(node_name),
      heater_id_(heater_id),
      temperature_K_(temperature_ini_K),
      capacity_J_K_(capacity_J_K),
      alpha_(alpha),
      area_m2_(area_m2),
      node_type_(node_type),
      normal_vector_b_(normal_vector_b) {
  AssertNodeParams();
  solar_radiation_W_ = 0;
}

Node::~Node() {}

double Node::CalcSolarRadiation_W(math::Vector<3> sun_direction_b, double solar_flux_W_m2) {
  double cos_theta = InnerProduct(sun_direction_b, normal_vector_b_);

  // calculate sun_power
  if (cos_theta > 0)
    solar_radiation_W_ = solar_flux_W_m2 * area_m2_ * alpha_ * cos_theta;
  else
    solar_radiation_W_ = 0;
  return solar_radiation_W_;
}

void Node::PrintParam(void) {
  string node_type_str = "";
  if (node_type_ == NodeType::kDiffusive) {
    node_type_str = "Diffusive";
  } else if (node_type_ == NodeType::kBoundary) {
    node_type_str = "Boundary";
  } else if (node_type_ == NodeType::kArithmetic) {
    node_type_str = "Arithmetic";
  }
  cout << "node_id: " << node_id_ << endl;
  cout << "  node_name    : " << node_name_ << endl;
  cout << "  temperature  : " << temperature_K_ << endl;
  cout << "  alpha        : " << alpha_ << endl;
  cout << "  capacity     : " << capacity_J_K_ << endl;
  cout << "  node type    : " << node_type_str << endl;
  cout << "  heater id    : " << heater_id_ << endl;
  cout << "  Normal Vector: ";
  for (size_t i = 0; i < 3; i++) {
    cout << normal_vector_b_[i] << " ";
  }
  cout << endl;
}

void Node::AssertNodeParams(void) {
  // Temperature must be larger than zero kelvin
  if (temperature_K_ < 0.0) {
    std::cerr << "[WARNING] node: temperature is less than zero [K]." << std::endl;
    std::cerr << "The value is set as 0.0." << std::endl;
    temperature_K_ = 0.0;
  }
  // Capacity must be larger than 0, use 0 when node is boundary or arithmetic
  if (capacity_J_K_ < 0.0) {
    std::cerr << "[WARNING] node: capacity is less than zero [J/K]." << std::endl;
    std::cerr << "The value is set as 0.0." << std::endl;
    capacity_J_K_ = 0.0;
  }
  // alpha must be between 0 and 1
  if (alpha_ < 0.0 || alpha_ > 1.0) {
    std::cerr << "[WARNING] node: alpha is over the range [0, 1]." << std::endl;
    std::cerr << "The value is set as 0.0." << std::endl;
    alpha_ = 0.0;
  }
  // Area must be larger than 0
  if (area_m2_ < 0.0) {
    std::cerr << "[WARNING] node: area is less than zero [m2]." << std::endl;
    std::cerr << "The value is set as 0.0." << std::endl;
    area_m2_ = 0.0;
  }
}

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
  math::Vector<3> normal_v_b;
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
