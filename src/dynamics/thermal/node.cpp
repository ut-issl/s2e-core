/**
 * @file node.cpp
 * @brief thermal calculation node
 */

#include "node.hpp"

#include <cmath>
#include <environment/global/physical_constants.hpp>

using namespace std;
using namespace libra;

Node::Node(const int node_id, const string node_label, const int node_type, const int heater_node_id, const double temperature_ini,
           const double capacity_ini, const double alpha, const double area, libra::Vector<3> normal_v_b)
    : node_id_(node_id),
      node_label_(node_label),
      heater_node_id_(heater_node_id),
      temperature_(temperature_ini),
      capacity_(capacity_ini),
      alpha_rad_(alpha),
      area_(area),
      node_type_(node_type),
      normal_v_b_(normal_v_b) {
  solar_radiation_ = 0;
}

Node::~Node() {}

double Node::K2deg(double kelvin) const {
  double temp = kelvin + environment::absolute_zero_degC;
  return temp;
}

int Node::GetNodeId(void) const { return node_id_; }

string Node::GetNodeLabel(void) const { return node_label_; }

int Node::GetHeaterNodeId(void) const { return heater_node_id_; }

double Node::GetTemperature_K(void) const { return temperature_; }

double Node::GetTemperature_deg(void) const {
  double temp = K2deg(temperature_);
  return temp;
}

double Node::GetCapacity(void) const { return capacity_; }

double Node::GetSolarRadiation(void) const { return solar_radiation_; }

int Node::GetNodeType(void) const { return node_type_; }

void Node::SetTemperature_K(double temp_K) { temperature_ = temp_K; }

double Node::CalcSolarRadiation(libra::Vector<3> sun_direction) {
  // FIXME: constants
  double R = 6.96E+8;                              // Distance from sun
  double T = 5778;                                 // sun surface temperature [K]
  double sigma = 5.67E-8;                          // stephan-boltzman constant
  double a = sun_direction.CalcNorm();             // distance from sun
  double S = pow((R / a), 2) * sigma * pow(T, 4);  // Solar Constant at s/c position S[W/m^2]
  double cos_theta = InnerProduct(sun_direction, normal_v_b_) / a / normal_v_b_.CalcNorm();

  // calculate sun_power
  if (cos_theta > 0)
    solar_radiation_ = S * area_ * alpha_rad_ * cos_theta;
  else
    solar_radiation_ = 0;
  return solar_radiation_;  //[W]
}

void Node::PrintParam(void) {
  cout << "node_id: " << node_id_ << endl;
  cout << "  node_label   : " << node_label_ << endl;
  cout << "  temperature  : " << temperature_ << endl;
  cout << "  alpha        : " << alpha_rad_ << endl;
  cout << "  capacity     : " << capacity_ << endl;
  cout << "  node type    : " << node_type_ << endl;
  cout << "  heater id    : " << heater_node_id_ << endl;
  cout << "  Normal Vector: ";
  for (int i = 0; i < 3; i++) {
    cout << normal_v_b_[i] << " ";
  }
  cout << endl;
}
