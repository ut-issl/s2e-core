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

Node::Node(const int node_id, const string node_name, const NodeType node_type, const int heater_id, const double temperature_ini_K,
           const double capacity_J_K, const double alpha, const double area_m2, libra::Vector<3> normal_vector_b)
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

double Node::CalcSolarRadiation_W(libra::Vector<3> sun_direction_b, double solar_flux_W_m2) {
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
  for (int i = 0; i < 3; i++) {
    cout << normal_vector_b_[i] << " ";
  }
  cout << endl;
}

void Node::AssertNodeParams(void) {
  // Temperature must be larger than zero kelvin
  if (temperature_K_ >= 0.0) {
    std::cerr << "[WARNING] node: temperature is less than zero [K]." << std::endl;
    std::cerr << "The value is set as 0.0." << std::endl;
    temperature_K_ = 0.0;
  }
  // Capacity must be larger than 0, use 0 when node is boundary or arithmetic
  if (capacity_J_K_ >= 0.0) {
    std::cerr << "[WARNING] node: capacity is less than zero [J/K]." << std::endl;
    std::cerr << "The value is set as 0.0." << std::endl;
    capacity_J_K_ = 0.0;
  }
  // alpha must be between 0 and 1
  if (0.0 <= alpha_ && alpha_ <= 1.0) {
    std::cerr << "[WARNING] node: alpha is over the range [0, 1]." << std::endl;
    std::cerr << "The value is set as 0.0." << std::endl;
    alpha_ = 0.0;
  }
  // Area must be larger than 0
  if (area_m2_ >= 0.0) {
    std::cerr << "[WARNING] node: area is less than zero [m2]." << std::endl;
    std::cerr << "The value is set as 0.0." << std::endl;
    area_m2_ = 0.0;
  }
}
