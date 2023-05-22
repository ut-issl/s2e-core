/**
 * @file node.cpp
 * @brief thermal calculation node
 */

#include "node.hpp"

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
      normal_vector_b_(normal_vector_b),
      node_type_(node_type) {
  solar_radiation_W_ = 0;
}

Node::~Node() {}

double Node::CalcSolarRadiation_W(libra::Vector<3> sun_direction_b) {
  // FIXME: constants
  double R = 6.96E+8;                              // Distance from sun
  double T = 5778;                                 // sun surface temperature [K]
  double sigma = 5.67E-8;                          // stephan-boltzman constant
  double a = sun_direction_b.CalcNorm();           // distance from sun
  double S = pow((R / a), 2) * sigma * pow(T, 4);  // Solar Constant at s/c position S[W/m^2]
  double cos_theta = InnerProduct(sun_direction_b, normal_vector_b_) / a / normal_vector_b_.CalcNorm();

  // calculate sun_power
  if (cos_theta > 0)
    solar_radiation_W_ = S * area_m2_ * alpha_ * cos_theta;
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
