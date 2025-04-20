/**
 * @file node.cpp
 * @brief thermal calculation node
 */

#include "node.hpp"

#include <cassert>
#include <cmath>
#include <environment/global/physical_constants.hpp>
#include <math_physics/math/constants.hpp>

using namespace std;
using namespace s2e::math;

namespace s2e::dynamics::thermal {

Node::Node(const size_t node_id, const string node_name, const NodeType node_type, const size_t heater_id, const int power_port_id,
           const double temperature_ini_K, const double capacity_J_K, const double alpha, const double epsilon, const double area_m2,
           math::Vector<3> normal_vector_b)
    : node_id_(node_id),
      node_name_(node_name),
      heater_id_(heater_id),
      power_port_id_(power_port_id),
      temperature_K_(temperature_ini_K),
      capacity_J_K_(capacity_J_K),
      alpha_(alpha),
      epsilon_(epsilon),
      area_m2_(area_m2),
      node_type_(node_type),
      normal_vector_b_(normal_vector_b) {
  AssertNodeParams();
  solar_radiation_W_ = 0.0;
  albedo_radiation_W_ = 0.0;
  earth_infrared_W_ = 0.0;
}

Node::~Node() {}

double Node::CalcSolarRadiation_W(math::Vector<3> sun_direction_b, double solar_flux_W_m2) {
  double cos_theta = InnerProduct(sun_direction_b, normal_vector_b_);

  // calculate sun_power
  if (cos_theta > 0.0)
    solar_radiation_W_ = solar_flux_W_m2 * area_m2_ * alpha_ * cos_theta;
  else
    solar_radiation_W_ = 0.0;
  return solar_radiation_W_;
}

double Node::CalcAlbedoRadiation_W(math::Vector<3> earth_position_b_m, math::Vector<3> sun_direction_b, double earth_albedo_W_m2) {
  math::Vector<3> sat2earth_direction_b = earth_position_b_m.CalcNormalizedVector();
  math::Vector<3> sat2sun_direction_b = sun_direction_b;

  math::Vector<3> vec_a = -sat2earth_direction_b;
  math::Vector<3> vec_b = (sat2sun_direction_b - sat2earth_direction_b).CalcNormalizedVector();

  // albedo radiation calculation; earth_albedo_W_m2 reflects the shadow coefficient.
  double cos_theta = InnerProduct(vec_a, vec_b);
  double lamda = acos(InnerProduct(sat2earth_direction_b, normal_vector_b_));
  double h = earth_position_b_m.CalcNorm() - environment::earth_equatorial_radius_m;
  double H = earth_position_b_m.CalcNorm() / environment::earth_equatorial_radius_m;
  double phi_m = asin(1.0 / H);
  double b = sqrt(H * H - 1.0);
  double view_factor;

  // Calc view factor
  // ref)POWER INPUT TO A SMALL FLAT PLATE FROM A DIFFUSELY RADIATING SPHERE WITH APPLICATION TO EARTH SATELLITES: THE SPINNING PLATE
  if (h < 1732e3) {
    if (lamda <= math::pi / 2.0 - phi_m) {
      view_factor = cos(lamda) / H / H;
    } else if (lamda <= math::pi / 2.0 + phi_m) {
      view_factor = 1.0 / 2.0 - 1.0 / math::pi * asin(b / (H * sin(lamda))) +
                    1.0 / (math::pi * pow(H, 2.0)) * (cos(lamda) * acos(-b / tan(lamda)) - b * sqrt(1.0 - pow(H * cos(lamda), 2.0)));
    } else {
      view_factor = 0.0;
    }
  } else {
    if (lamda < math::pi / 2.0) {
      // Consider in terms of steradian
      view_factor = 0.25 / H / H;
    } else {
      view_factor = 0.0;
    }
  }
  // Banister's approximation. ref) RADIATION GEOMETRY FACTOR BETWEEN THE EARTH AND A SATELLITE
  if (cos_theta > 0.0) {
    // TODO: correlate the value of the exponent with the view factor
    view_factor *= pow(cos_theta, 3.0);
  } else {
    view_factor = 0.0;
  }

  // albedo radiation calculation; earth_albedo_W_m2 reflects the shadow coefficient.
  albedo_radiation_W_ = earth_albedo_W_m2 * area_m2_ * alpha_ * view_factor;

  return albedo_radiation_W_;
}

double Node::CalcEarthInfraredRadiation_W(math::Vector<3> earth_position_b_m, double earth_infrared_W_m2) {
  math::Vector<3> earth_direction_b = earth_position_b_m.CalcNormalizedVector();

  double lamda = acos(InnerProduct(earth_direction_b, normal_vector_b_));
  double h = earth_position_b_m.CalcNorm() - environment::earth_equatorial_radius_m;
  double H = earth_position_b_m.CalcNorm() / environment::earth_equatorial_radius_m;
  double phi_m = asin(1.0 / H);
  double b = sqrt(H * H - 1.0);
  double view_factor;

  // Calc view factor
  // ref)POWER INPUT TO A SMALL FLAT PLATE FROM A DIFFUSELY RADIATING SPHERE WITH APPLICATION TO EARTH SATELLITES: THE SPINNING PLATE
  if (h < 1732e3) {
    if (lamda <= math::pi / 2.0 - phi_m) {
      view_factor = cos(lamda) / H / H;
    } else if (lamda <= math::pi / 2.0 + phi_m) {
      view_factor = 1.0 / 2.0 - 1.0 / math::pi * asin(b / (H * sin(lamda))) +
                    1.0 / (math::pi * pow(H, 2.0)) * (cos(lamda) * acos(-b / tan(lamda)) - b * sqrt(1.0 - pow(H * cos(lamda), 2.0)));
    } else {
      view_factor = 0.0;
    }
  } else {
    if (lamda < math::pi / 2.0) {
      // Consider in terms of steradian
      view_factor = 0.25 / H / H;
    } else {
      view_factor = 0.0;
    }
  }

  // earth infrared radiation calculation
  earth_infrared_W_ = earth_infrared_W_m2 * area_m2_ * epsilon_ * view_factor;

  return earth_infrared_W_;
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
  cout << "  epsilon         : " << epsilon_ << endl;
  cout << "  capacity     : " << capacity_J_K_ << endl;
  cout << "  node type    : " << node_type_str << endl;
  cout << "  heater id    : " << heater_id_ << endl;
  cout << "  power port id: " << power_port_id_ << endl;
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
  // epsilon must be between 0 and 1
  if (epsilon_ < 0.0 || epsilon_ > 1.0) {
    std::cerr << "[WARNING] node: epsilon is over the range [0, 1]." << std::endl;
    std::cerr << "The value is set as 0.0." << std::endl;
    epsilon_ = 0.0;
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
column 5: power port id
column 6: capacity
column 7: alpha
column 8: epsilon
column 9: area
column 10,11,12: normal vector of surface(body frame)
column 13: initial temperature(K)

First row is for Header, data begins from the second row
Ex.
Node_id,Node_label,node_type,heater_id,power_port_id,capacity,alpha,epsilon,area,normal_v_b_x,normal_v_b_y,normal_v_b_z,initial_temperature
0,      BUS,       0,        1,        -1,           880,     0.2,  0.7,    0.06,1,           0,           0,           300
1,      SAP,       0,        0,        -1,           100,     0.8,  0.8,    0.02,0,           0,           1,           250
2,      SPACE,     1,        0,        -1,           0,       0,    0,      0,   0,           0,           0,           2.73

Be sure to include at least one boundary node to avoid divergence
*/

Node InitNode(const std::vector<std::string>& node_str) {
  using std::stod;
  using std::stoi;

  size_t node_str_size_defined = 13;                 // Correct size of node_str
  assert(node_str.size() == node_str_size_defined);  // Check if size of node_str is correct

  size_t node_id = 0;               // node number
  std::string node_label = "temp";  // node name
  size_t node_type_int = 0;         // node type
  size_t heater_id = 0;             // heater node index
  int power_port_id = -1;           // power port index
  double temperature_K = 0.0;       // [K]
  double capacity_J_K = 0.0;        // [J/K]
  double alpha = 0.0;               // []
  double epsilon = 0.0;             // []
  double area_m2 = 0.0;             // [m^2]

  // Index to read from node_str for each parameter
  size_t index_node_id = 0;
  size_t index_node_label = 1;
  size_t index_node_type = 2;
  size_t index_heater_id = 3;
  size_t index_power_port_id = 4;
  size_t index_capacity = 5;
  size_t index_alpha = 6;
  size_t index_epsilon = 7;
  size_t index_area = 8;
  size_t index_normal_v_b_head = 9;
  size_t index_temperature = 12;

  node_id = stoi(node_str[index_node_id]);
  node_label = node_str[index_node_label];
  node_type_int = stoi(node_str[index_node_type]);
  heater_id = stoi(node_str[index_heater_id]);
  power_port_id = stoi(node_str[index_power_port_id]);
  capacity_J_K = stod(node_str[index_capacity]);
  alpha = stod(node_str[index_alpha]);
  epsilon = stod(node_str[index_epsilon]);
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
  Node node(node_id, node_label, node_type, heater_id, power_port_id, temperature_K, capacity_J_K, alpha, epsilon, area_m2, normal_v_b);
  return node;
}

}  // namespace s2e::dynamics::thermal
