#include "Node.h"

#include <cmath>
#include <Environment/Global/PhysicalConstants.hpp>

using namespace std;
using namespace libra;

Node::Node(const int node_id, const string node_label, const int heater_node_id, const double temperature_ini, const double capacity_ini,
           const double internal_heat_ini, const double alpha, const double area, Vector<3> normal_v_b)
    : node_id_(node_id),
      node_label_(node_label),
      heater_node_id_(heater_node_id),
      temperature_(temperature_ini),
      capacity_(capacity_ini),
      internal_heat_(internal_heat_ini),
      alpha_(alpha),
      area_(area),
      normal_v_b_(normal_v_b) {
  solar_radiation_ = 0;
}

Node::~Node() {}

double Node::K2deg(double kelvin) const {
  double temp = kelvin + libra::absolute_zero_degC;
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

double Node::GetInternalHeat(void) const { return internal_heat_; }

double Node::GetSolarRadiation(void) const { return solar_radiation_; }

void Node::SetTemperature_K(double temp_K) { temperature_ = temp_K; }

void Node::SetInternalHeat(double heat_power) {
  internal_heat_ = heat_power;  // [W]
}

double Node::CalcSolarRadiation(Vector<3> sun_direction) {
  // FIXME: constants
  double R = 6.96E+8;                              // Distance from sun
  double T = 5778;                                 // sun surface temperature [K]
  double sigma = 5.67E-8;                          // stephan-boltzman constant
  double a = norm(sun_direction);                  // distance from sun
  double S = pow((R / a), 2) * sigma * pow(T, 4);  // Solar Constant at s/c position S[W/m^2]
  double cos_theta = inner_product(sun_direction, normal_v_b_) / a / norm(normal_v_b_);

  // calculate sun_power
  if (cos_theta > 0)
    solar_radiation_ = S * area_ * alpha_ * cos_theta;
  else
    solar_radiation_ = 0;
  return solar_radiation_;  //[W]
}

void Node::PrintParam(void) {
  cout << "node_id: " << node_id_ << endl;
  cout << "  node_label   : " << node_label_ << endl;
  cout << "  temperature  : " << temperature_ << endl;
  cout << "  alpha        : " << alpha_ << endl;
  cout << "  capacity     : " << capacity_ << endl;
  cout << "  internal heat; " << internal_heat_ << endl;
  cout << "  Normal Vector: ";
  for (int i = 0; i < 3; i++) {
    cout << normal_v_b_[i] << " ";
  }
  cout << endl;
}
