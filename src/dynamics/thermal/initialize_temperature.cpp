/**
 * @file initialize_temperature.cpp
 * @brief Initialize function for temperature
 */

#include "initialize_temperature.hpp"

#include <cassert>
#include <environment/global/simulation_time.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <string>

#include "initialize_heater.hpp"
#include "initialize_heatload.hpp"
#include "initialize_node.hpp"

/* Import node properties, heatload_list and Cij/Rij Datas by reading CSV File (node.csv,
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

/* Parameters
   mainIni    : main Ini file(simBase.ini)
   file_path  : directory of Thermal Input files(read from SimBase.ini)
   propstep   : time step interval for temperature propagation integration(read
   from SimBase.ini) mainstepSec: time step interval for Main Routin integration
   (= temperature.end_time_)
 */

using std::string;
using std::vector;

Temperature* InitTemperature(const std::string file_name, const double rk_prop_step_s) {
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
  unsigned int node_num = 1;
  unsigned int heatload_num = 1;
  unsigned int heater_num = 1;

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
    Recall that Nodes_num are given to this function only to reseve memory*/

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
    Recall that Nodes_num are given to this function only to reseve memory*/

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
    Recall that heater_num are given to this function only to reseve memory*/

  heater_num = heater_str_list.size() - 1;                                             // First Row is for Header(not data)
  heater_list.reserve(heater_num);                                                     // reserve memory
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
                                rk_prop_step_s, is_calc_enabled, solar_calc_setting, debug);
  return temperature;
}
