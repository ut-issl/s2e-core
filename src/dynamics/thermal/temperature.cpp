/**
 * @file temperature.hpp
 * @brief Initialize temperature
 */

#include "temperature.hpp"

#include <cassert>
#include <cmath>
#include <environment/global/physical_constants.hpp>
#include <environment/global/simulation_time.hpp>
#include <setting_file_reader/initialize_file_access.hpp>
#include <utilities/macros.hpp>

using namespace std;

Temperature::Temperature(const vector<vector<double>> conductance_matrix_W_K, const vector<vector<double>> radiation_matrix_m2, vector<Node> nodes,
                         vector<Heatload> heatloads, vector<Heater> heaters, vector<HeaterController> heater_controllers, const size_t node_num,
                         const double propagation_step_s, const SolarRadiationPressureEnvironment* srp_environment, const EarthAlbedo* earth_albedo,
                         const bool is_calc_enabled, const SolarCalcSetting solar_calc_setting, const bool debug)
    : conductance_matrix_W_K_(conductance_matrix_W_K),
      radiation_matrix_m2_(radiation_matrix_m2),
      nodes_(nodes),
      heatloads_(heatloads),
      heaters_(heaters),
      heater_controllers_(heater_controllers),
      node_num_(node_num),
      propagation_step_s_(propagation_step_s),
      srp_environment_(srp_environment),
      earth_albedo_(earth_albedo),
      is_calc_enabled_(is_calc_enabled),
      solar_calc_setting_(solar_calc_setting),
      debug_(debug) {
  propagation_time_s_ = 0;
  if (debug_) {
    PrintParams();
  }
}

Temperature::Temperature() {
  node_num_ = 0;
  propagation_step_s_ = 0.0;
  propagation_time_s_ = 0.0;
  solar_calc_setting_ = SolarCalcSetting::kDisable;
  is_calc_enabled_ = false;
  debug_ = false;
}

Temperature::~Temperature() {}

void Temperature::Propagate(math::Vector<3> sun_position_b_m, const double time_end_s) {
  if (!is_calc_enabled_) return;
  math::Vector<3> sun_direction_b = sun_position_b_m.CalcNormalizedVector();
  while (time_end_s - propagation_time_s_ - propagation_step_s_ > 1.0e-6) {
    CalcRungeOneStep(propagation_time_s_, propagation_step_s_, sun_direction_b, node_num_);
    propagation_time_s_ += propagation_step_s_;
  }
  CalcRungeOneStep(propagation_time_s_, time_end_s - propagation_time_s_, sun_direction_b, node_num_);
  propagation_time_s_ = time_end_s;
  UpdateHeaterStatus();

  if (debug_) {
    cout << fixed;
    cout << "Time: " << time_end_s << "  Temp:  ";
    for (auto itr = nodes_.begin(); itr != nodes_.end(); ++itr) {
      cout << setprecision(4) << itr->GetTemperature_degC() << "  ";
    }
    cout << "SolarR:  ";
    for (auto itr = nodes_.begin(); itr != nodes_.end(); ++itr) {
      cout << setprecision(4) << itr->GetSolarRadiation_W() << "  ";
    }
    cout << "SunDir:  ";
    for (size_t i = 0; i < 3; i++) {
      cout << setprecision(3) << sun_direction_b[i] << "  ";
    }
    cout << "Heatload:  ";
    for (auto itr = heatloads_.begin(); itr != heatloads_.end(); ++itr) {
      cout << setprecision(3) << itr->GetTotalHeatload_W() << "  ";
    }
    cout << endl;
  }
}

void Temperature::Propagate(const LocalCelestialInformation* local_celestial_information, const double time_end_s) {
  if (!is_calc_enabled_) return;
  while (time_end_s - propagation_time_s_ - propagation_step_s_ > 1.0e-6) {
    CalcRungeOneStep(propagation_time_s_, propagation_step_s_, local_celestial_information, node_num_);
    propagation_time_s_ += propagation_step_s_;
  }
  CalcRungeOneStep(propagation_time_s_, time_end_s - propagation_time_s_, local_celestial_information, node_num_);
  propagation_time_s_ = time_end_s;
  UpdateHeaterStatus();

  if (debug_) {
    cout << fixed;
    cout << "Time: " << time_end_s << "  Temp:  ";
    for (auto itr = nodes_.begin(); itr != nodes_.end(); ++itr) {
      cout << setprecision(4) << itr->GetTemperature_degC() << "  ";
    }
    cout << "SolarR:  ";
    for (auto itr = nodes_.begin(); itr != nodes_.end(); ++itr) {
      cout << setprecision(4) << itr->GetSolarRadiation_W() << "  ";
    }
    math::Vector<3> sun_direction_b = local_celestial_information->GetPositionFromSpacecraft_b_m("SUN").CalcNormalizedVector();
    cout << "SunDir:  ";
    for (size_t i = 0; i < 3; i++) {
      cout << setprecision(3) << sun_direction_b[i] << "  ";
    }
    cout << "ShadowCoefficient:  " << setprecision(4) << srp_environment_->GetShadowCoefficient() << "  ";
    cout << "EarthAlbedoR:  ";
    for (auto itr = nodes_.begin(); itr != nodes_.end(); ++itr) {
      cout << setprecision(4) << itr->GetAlbedoRadiation_W() << "  ";
    }
    math::Vector<3> earth_direction_b = local_celestial_information->GetPositionFromSpacecraft_b_m("EARTH").CalcNormalizedVector();
    cout << "EarthDir:  ";
    for (size_t i = 0; i < 3; i++) {
      cout << setprecision(3) << earth_direction_b[i] << "  ";
    }
    cout << "Heatload:  ";
    for (auto itr = heatloads_.begin(); itr != heatloads_.end(); ++itr) {
      cout << setprecision(3) << itr->GetTotalHeatload_W() << "  ";
    }
    cout << endl;
  }
}

void Temperature::CalcRungeOneStep(double time_now_s, double time_step_s, math::Vector<3> sun_direction_b, size_t node_num) {
  vector<double> temperatures_now_K(node_num);
  for (size_t i = 0; i < node_num; i++) {
    temperatures_now_K[i] = nodes_[i].GetTemperature_K();
  }

  vector<double> k1(node_num), k2(node_num), k3(node_num), k4(node_num);
  vector<double> xk2(node_num), xk3(node_num), xk4(node_num);

  k1 = CalcTemperatureDifferentials(temperatures_now_K, time_now_s, sun_direction_b, node_num);
  for (size_t i = 0; i < node_num; i++) {
    xk2[i] = temperatures_now_K[i] + (time_step_s / 2.0) * k1[i];
  }

  k2 = CalcTemperatureDifferentials(xk2, (time_now_s + time_step_s / 2.0), sun_direction_b, node_num);
  for (size_t i = 0; i < node_num; i++) {
    xk3[i] = temperatures_now_K[i] + (time_step_s / 2.0) * k2[i];
  }

  k3 = CalcTemperatureDifferentials(xk3, (time_now_s + time_step_s / 2.0), sun_direction_b, node_num);
  for (size_t i = 0; i < node_num; i++) {
    xk4[i] = temperatures_now_K[i] + time_step_s * k3[i];
  }

  k4 = CalcTemperatureDifferentials(xk4, (time_now_s + time_step_s), sun_direction_b, node_num);

  vector<double> temperatures_next_K(node_num);
  for (size_t i = 0; i < node_num; i++) {
    temperatures_next_K[i] = temperatures_now_K[i] + (time_step_s / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }

  for (size_t i = 0; i < node_num; i++) {
    nodes_[i].SetTemperature_K(temperatures_next_K[i]);
  }
}

void Temperature::CalcRungeOneStep(double time_now_s, double time_step_s, const LocalCelestialInformation* local_celestial_information,
                                   size_t node_num) {
  vector<double> temperatures_now_K(node_num);
  for (size_t i = 0; i < node_num; i++) {
    temperatures_now_K[i] = nodes_[i].GetTemperature_K();
  }

  vector<double> k1(node_num), k2(node_num), k3(node_num), k4(node_num);
  vector<double> xk2(node_num), xk3(node_num), xk4(node_num);

  k1 = CalcTemperatureDifferentials(temperatures_now_K, time_now_s, local_celestial_information, node_num);
  for (size_t i = 0; i < node_num; i++) {
    xk2[i] = temperatures_now_K[i] + (time_step_s / 2.0) * k1[i];
  }

  k2 = CalcTemperatureDifferentials(xk2, (time_now_s + time_step_s / 2.0), local_celestial_information, node_num);
  for (size_t i = 0; i < node_num; i++) {
    xk3[i] = temperatures_now_K[i] + (time_step_s / 2.0) * k2[i];
  }

  k3 = CalcTemperatureDifferentials(xk3, (time_now_s + time_step_s / 2.0), local_celestial_information, node_num);
  for (size_t i = 0; i < node_num; i++) {
    xk4[i] = temperatures_now_K[i] + time_step_s * k3[i];
  }

  k4 = CalcTemperatureDifferentials(xk4, (time_now_s + time_step_s), local_celestial_information, node_num);

  vector<double> temperatures_next_K(node_num);
  for (size_t i = 0; i < node_num; i++) {
    temperatures_next_K[i] = temperatures_now_K[i] + (time_step_s / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }

  for (size_t i = 0; i < node_num; i++) {
    nodes_[i].SetTemperature_K(temperatures_next_K[i]);
  }
}

vector<double> Temperature::CalcTemperatureDifferentials(vector<double> temperatures_K, double t, math::Vector<3> sun_direction_b, size_t node_num) {
  // TODO: consider the following unused arguments are really needed
  UNUSED(temperatures_K);

  vector<double> differentials_K_s(node_num);
  for (size_t i = 0; i < node_num; i++) {
    heatloads_[i].SetElapsedTime_s(t);
    if (nodes_[i].GetNodeType() == NodeType::kDiffusive) {
      double solar_flux_W_m2 = srp_environment_->GetPowerDensity_W_m2();
      if (solar_calc_setting_ == SolarCalcSetting::kEnable) {
        double solar_radiation_W = nodes_[i].CalcSolarRadiation_W(sun_direction_b, solar_flux_W_m2);
        heatloads_[i].SetSolarHeatload_W(solar_radiation_W);
      }
      double heater_power_W = GetHeaterPower_W(i);
      heatloads_[i].SetHeaterHeatload_W(heater_power_W);
      heatloads_[i].CalcInternalHeatload();
      heatloads_[i].UpdateTotalHeatload();
      double total_heatload_W = heatloads_[i].GetTotalHeatload_W();  // Total heatload (solar + internal + heater)[W]

      double conductive_heat_input_W = 0;
      double radiative_heat_input_W = 0;
      for (size_t j = 0; j < node_num; j++) {
        conductive_heat_input_W += conductance_matrix_W_K_[i][j] * (nodes_[j].GetTemperature_K() - nodes_[i].GetTemperature_K());
        radiative_heat_input_W += environment::stefan_boltzmann_constant_W_m2K4 * radiation_matrix_m2_[i][j] *
                                  (pow(nodes_[j].GetTemperature_K(), 4) - pow(nodes_[i].GetTemperature_K(), 4));
      }
      double total_heat_input_W = conductive_heat_input_W + radiative_heat_input_W + total_heatload_W;
      differentials_K_s[i] = total_heat_input_W / nodes_[i].GetCapacity_J_K();
    } else if (nodes_[i].GetNodeType() == NodeType::kBoundary) {
      differentials_K_s[i] = 0;
    }
  }
  return differentials_K_s;
}

vector<double> Temperature::CalcTemperatureDifferentials(vector<double> temperatures_K, double t,
                                                         const LocalCelestialInformation* local_celestial_information, size_t node_num) {
  // TODO: consider the following unused arguments are really needed
  UNUSED(temperatures_K);

  math::Vector<3> sun_direction_b = local_celestial_information->GetPositionFromSpacecraft_b_m("SUN").CalcNormalizedVector();
  vector<double> differentials_K_s(node_num);
  for (size_t i = 0; i < node_num; i++) {
    heatloads_[i].SetElapsedTime_s(t);
    if (nodes_[i].GetNodeType() == NodeType::kDiffusive) {
      double solar_flux_W_m2 = srp_environment_->GetPowerDensity_W_m2();
      if (solar_calc_setting_ == SolarCalcSetting::kEnable) {
        double solar_radiation_W = nodes_[i].CalcSolarRadiation_W(sun_direction_b, solar_flux_W_m2);
        math::Vector<3> earth_position_b_m = local_celestial_information->GetPositionFromSpacecraft_b_m("EARTH");
        double albedo_radiation_W = nodes_[i].CalcAlbedoRadiation_W(earth_position_b_m, earth_albedo_->GetEarthAlbedoRadiationPower_W_m2());
        heatloads_[i].SetAlbedoHeatload_W(albedo_radiation_W);
        heatloads_[i].SetSolarHeatload_W(solar_radiation_W);
      }
      double heater_power_W = GetHeaterPower_W(i);
      heatloads_[i].SetHeaterHeatload_W(heater_power_W);
      heatloads_[i].CalcInternalHeatload();
      heatloads_[i].UpdateTotalHeatload();
      double total_heatload_W = heatloads_[i].GetTotalHeatload_W();  // Total heatload (solar + internal + heater)[W]

      double conductive_heat_input_W = 0;
      double radiative_heat_input_W = 0;
      for (size_t j = 0; j < node_num; j++) {
        conductive_heat_input_W += conductance_matrix_W_K_[i][j] * (nodes_[j].GetTemperature_K() - nodes_[i].GetTemperature_K());
        radiative_heat_input_W += environment::stefan_boltzmann_constant_W_m2K4 * radiation_matrix_m2_[i][j] *
                                  (pow(nodes_[j].GetTemperature_K(), 4) - pow(nodes_[i].GetTemperature_K(), 4));
      }
      double total_heat_input_W = conductive_heat_input_W + radiative_heat_input_W + total_heatload_W;
      differentials_K_s[i] = total_heat_input_W / nodes_[i].GetCapacity_J_K();
    } else if (nodes_[i].GetNodeType() == NodeType::kBoundary) {
      differentials_K_s[i] = 0;
    }
  }
  return differentials_K_s;
}

double Temperature::GetHeaterPower_W(size_t node_id) {
  size_t heater_id = nodes_[node_id].GetHeaterId();
  double heater_power_W = 0.0;
  if (heater_id > 0) {
    heater_power_W = heaters_[heater_id - 1].GetPowerOutput_W();
  }
  return heater_power_W;
}

void Temperature::UpdateHeaterStatus(void) {
  for (auto itr = nodes_.begin(); itr != nodes_.end(); ++itr) {
    size_t heater_id = itr->GetHeaterId();
    if (heater_id > 0) {
      double temperature_degC = itr->GetTemperature_degC();
      Heater* p_heater = &heaters_[heater_id - 1];
      heater_controllers_[heater_id - 1].ControlHeater(p_heater, temperature_degC);
    }
  }
}

string Temperature::GetLogHeader() const {
  string str_tmp = "";
  for (size_t i = 0; i < node_num_; i++) {
    // Do not retrieve boundary node values
    if (nodes_[i].GetNodeType() != NodeType::kBoundary) {
      string str_node = "temp_" + to_string(nodes_[i].GetNodeId()) + " (" + nodes_[i].GetNodeName() + ")";
      str_tmp += WriteScalar(str_node, "deg");
    }
  }
  for (size_t i = 0; i < node_num_; i++) {
    // Do not retrieve boundary node values
    if (nodes_[i].GetNodeType() != NodeType::kBoundary) {
      string str_node = "heat_" + to_string(nodes_[i].GetNodeId()) + " (" + nodes_[i].GetNodeName() + ")";
      str_tmp += WriteScalar(str_node, "W");
    }
  }
  return str_tmp;
}

string Temperature::GetLogValue() const {
  string str_tmp = "";
  for (size_t i = 0; i < node_num_; i++) {
    // Do not retrieve boundary node values
    if (nodes_[i].GetNodeType() != NodeType::kBoundary) {
      str_tmp += WriteScalar(nodes_[i].GetTemperature_degC());
    }
  }
  for (size_t i = 0; i < node_num_; i++) {
    // Do not retrieve boundary node values
    if (nodes_[i].GetNodeType() != NodeType::kBoundary) {
      str_tmp += WriteScalar(heatloads_[i].GetTotalHeatload_W());
    }
  }
  return str_tmp;
}

void Temperature::PrintParams(void) {
  cout << "< Print Thermal Parameters >" << endl;
  cout << "IsCalcEnabled: " << is_calc_enabled_ << endl;
  cout << "V nodes:" << endl;
  for (auto itr = nodes_.begin(); itr != nodes_.end(); ++itr) {
    itr->PrintParam();
  }
  cout << "V heaters:" << endl;
  for (auto itr = heaters_.begin(); itr != heaters_.end(); ++itr) {
    itr->PrintParam();
  }
  cout << std::fixed;
  cout << "Cij:" << endl;
  for (size_t i = 0; i < (node_num_); i++) {
    for (size_t j = 0; j < (node_num_); j++) {
      cout << std::setprecision(4) << conductance_matrix_W_K_[i][j] << "  ";
    }
    cout << endl;
  }
  cout << "Rij:" << endl;
  for (size_t i = 0; i < (node_num_); i++) {
    for (size_t j = 0; j < (node_num_); j++) {
      cout << std::setprecision(4) << radiation_matrix_m2_[i][j] << "  ";
    }
    cout << endl;
  }
  cout << "**************************************" << endl;
}

/* Import node properties, heatload_list and Cij/Rij Data by reading CSV File (node.csv,
heatload.csv, cij.csv, rij.csv) Detailed process of reading node properties from CSV File, and
CSV file formats of node properties is written in Init_Node.cpp

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

[File Formats of Heatload.csv]
column 1 : Node_id(int)
column 2-: time and heat input data(double)

First row is time data

[File Formats of Cij.csv and Rij.Csv]
 Cij and Rij information is written in Row i, column j
 Ex.
    0.5 0.3 0.2
    0.1 0.4 0.1
    0.2 0.2 0.5

    →C12 = 0.3
  Cij units: W/K
  Rij units: m^2 (Equivalent to Ai * Bij * eps_i * eps_j)
*/

using std::string;
using std::vector;

Temperature* InitTemperature(const std::string file_name, const double rk_prop_step_s, const SolarRadiationPressureEnvironment* srp_environment,
                             const EarthAlbedo* earth_albedo) {
  auto mainIni = IniAccess(file_name);

  vector<Node> node_list;
  vector<Heater> heater_list;
  vector<HeaterController> heater_controller_list;
  vector<Heatload> heatload_list;
  vector<vector<double>> conductance_matrix;
  vector<vector<double>> radiation_matrix;
  vector<vector<string>> node_str_list;      // string vector of node property data
  vector<vector<string>> heater_str_list;    // string vector of heater property data
  vector<vector<string>> heatload_str_list;  // string vector of heatload property data
  size_t node_num = 1;
  size_t heatload_num = 1;
  size_t heater_num = 1;

  bool is_calc_enabled = mainIni.ReadEnable("THERMAL", "calculation");
  if (is_calc_enabled == false) {
    // Return here to avoid CSV file reading
    Temperature* temperature;
    temperature = new Temperature();
    return temperature;
  }

  // read ini-file settings
  string file_path = mainIni.ReadString("THERMAL", "thermal_file_directory");
  SolarCalcSetting solar_calc_setting;

  bool is_solar_calc_enabled = mainIni.ReadEnable("THERMAL", "solar_calc_setting");
  if (is_solar_calc_enabled) {
    solar_calc_setting = SolarCalcSetting::kEnable;
  } else {
    solar_calc_setting = SolarCalcSetting::kDisable;
  }

  bool debug = mainIni.ReadEnable("THERMAL", "debug");

  // Read Heatloads from CSV File
  string filepath_heatload = file_path + "heatload.csv";
  IniAccess conf_heatload(filepath_heatload);
  conf_heatload.ReadCsvString(heatload_str_list, 100);
  /*since we don't know the number of node_list yet, set node_num=100 temporary.
    Recall that Nodes_num are given to this function only to reserve memory*/

  heatload_num = heatload_str_list.size() - 1;
  auto times_itr = heatload_str_list.begin();  // First Row is Time Data
  for (auto itr = heatload_str_list.begin() + 1; itr != heatload_str_list.end(); ++itr) {
    heatload_list.push_back(InitHeatload(*times_itr, *itr));
  }

  // Read Node Properties from CSV File
  string filepath_node = file_path + "node.csv";
  IniAccess conf_node(filepath_node);
  conf_node.ReadCsvString(node_str_list, 100);
  /*since we don't know the number of node_list yet, set node_num=100 temporary.
    Recall that Nodes_num are given to this function only to reserve memory*/

  node_num = node_str_list.size() - 1;                                             // First Row is for Header(not data)
  node_list.reserve(node_num);                                                     // reserve memory
  for (auto itr = node_str_list.begin() + 1; itr != node_str_list.end(); ++itr) {  // first row is for labels
    node_list.push_back(InitNode(*itr));
  }

  assert(node_num == heatload_num);  // Number of nodes and heatload lists must be the same

  // Read Heater Properties from CSV File
  string filepath_heater = file_path + "heaters.csv";
  IniAccess conf_heater(filepath_heater);
  conf_heater.ReadCsvString(heater_str_list, 100);
  /*since we don't know the number of heater_list yet, set heater_num=100 temporary.
    Recall that heater_num are given to this function only to reserve memory*/

  heater_num = heater_str_list.size() - 1;  // First Row is for Header(not data)
  heater_list.reserve(heater_num);          // reserve memory
  heater_controller_list.reserve(heater_num);
  for (auto itr = heater_str_list.begin() + 1; itr != heater_str_list.end(); ++itr) {  // first row is for labels
    heater_list.push_back(InitHeater(*itr));
    heater_controller_list.push_back(InitHeaterController(*itr));
  }

  // Read Cij,Rij data from CSV File
  string filepath_cij = file_path + "cij.csv";
  string filepath_rij = file_path + "rij.csv";
  IniAccess conf_cij(filepath_cij);
  IniAccess conf_rij(filepath_rij);
  conf_cij.ReadCsvDoubleWithHeader(conductance_matrix, node_num, 1, 1);
  conf_rij.ReadCsvDoubleWithHeader(radiation_matrix, node_num, 1, 1);

  assert(conductance_matrix.size() == node_num);                      // Dimension must be same as node_num
  assert(radiation_matrix.size() == node_num);                        // Dimension must be same as node_num
  assert(conductance_matrix.size() == conductance_matrix[0].size());  // Must be square matrix
  assert(radiation_matrix.size() == radiation_matrix[0].size());      // Must be square matrix

  Temperature* temperature;
  temperature = new Temperature(conductance_matrix, radiation_matrix, node_list, heatload_list, heater_list, heater_controller_list, node_num,
                                rk_prop_step_s, srp_environment, earth_albedo, is_calc_enabled, solar_calc_setting, debug);
  return temperature;
}
