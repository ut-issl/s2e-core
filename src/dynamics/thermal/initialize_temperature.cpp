/**
 * @file initialize_temperature.cpp
 * @brief Initialize function for temperature
 */

#include "initialize_temperature.hpp"

#include <environment/global/simulation_time.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <string>

#include "initialize_heater.hpp"
#include "initialize_heatload.hpp"
#include "initialize_node.hpp"

/* Import node properties, heatloads and Cij/Rij Datas by reading CSV File (node.csv,
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

Temperature* InitTemperature(const std::string file_name, const double rk_prop_step_sec) {
  auto mainIni = IniAccess(file_name);

  vector<Node> vnodes;
  vector<Heater> vheaters;
  vector<Heatload> vheatloads;
  vector<vector<double>> cij;
  vector<vector<double>> rij;
  vector<vector<string>> vnodestr;      // string vector of node property data
  vector<vector<string>> vheaterstr;    // string vector of heater property data
  vector<vector<string>> vheatloadstr;  // string vector of heatload property data
  int nodes_num = 1;
  int heater_num = 1;

  bool is_calc_enabled = mainIni.ReadEnable("THERMAL", "calculation");
  if (is_calc_enabled == false) {
    // Return here to avoid CSV file reading
    Temperature* temperature;
    temperature = new Temperature();
    return temperature;
  }

  // read ini-file settings
  string file_path = mainIni.ReadString("THERMAL", "thermal_file_directory");
  string solar_calc_setting_str = mainIni.ReadString("THERMAL", "solar_calc_setting");
  SolarCalcSetting solar_calc_setting;

  if (solar_calc_setting_str == "ENABLE") {
    solar_calc_setting = SolarCalcSetting::kEnable;
  } else {
    solar_calc_setting = SolarCalcSetting::kDisable;
  }

  bool debug = mainIni.ReadBoolean("THERMAL", "debug");

  // Read Heatloads from CSV File
  string filepath_heatload = file_path + "heatload.csv";
  IniAccess conf_heatload(filepath_heatload);
  conf_heatload.ReadCsvString(vheatloadstr, 100);
  /*since we don't know the number of nodes yet, set nodes_num=100 temporary.
    Recall that Nodes_num are given to this function only to reseve memory*/

  auto times_itr = vheatloadstr.begin();  // First Row is Time Data
  for (auto itr = vheatloadstr.begin() + 1; itr != vheatloadstr.end(); ++itr) {
    vheatloads.push_back(InitHeatload(*times_itr, *itr));
  }

  // Read Node Properties from CSV File
  string filepath_node = file_path + "node.csv";
  IniAccess conf_node(filepath_node);
  conf_node.ReadCsvString(vnodestr, 100);
  /*since we don't know the number of nodes yet, set nodes_num=100 temporary.
    Recall that Nodes_num are given to this function only to reseve memory*/

  nodes_num = vnodestr.size() - 1;                                       // First Row is for Header(not data)
  vnodes.reserve(nodes_num);                                             // reserve memory
  for (auto itr = vnodestr.begin() + 1; itr != vnodestr.end(); ++itr) {  // first row is for labels
    vnodes.push_back(InitNode(*itr));
  }

  // Read Heater Properties from CSV File
  string filepath_heater = file_path + "heaters.csv";
  IniAccess conf_heater(filepath_heater);
  conf_heater.ReadCsvString(vheaterstr, 100);
  /*since we don't know the number of heaters yet, set heater_num=100 temporary.
    Recall that heater_num are given to this function only to reseve memory*/

  heater_num = vheaterstr.size() - 1;                                        // First Row is for Header(not data)
  vheaters.reserve(heater_num);                                              // reserve memory
  for (auto itr = vheaterstr.begin() + 1; itr != vheaterstr.end(); ++itr) {  // first row is for labels
    vheaters.push_back(InitHeater(*itr));
  }

  // Read Cij,Rij data from CSV File
  string filepath_cij = file_path + "cij.csv";
  string filepath_rij = file_path + "rij.csv";
  IniAccess conf_cij(filepath_cij);
  IniAccess conf_rij(filepath_rij);
  conf_cij.ReadCsvDoubleWithHeader(cij, nodes_num, 1, 1);
  conf_rij.ReadCsvDoubleWithHeader(rij, nodes_num, 1, 1);

  Temperature* temperature;
  temperature = new Temperature(cij, rij, vnodes, vheatloads, vheaters, nodes_num, rk_prop_step_sec, is_calc_enabled, solar_calc_setting, debug);
  return temperature;
}
