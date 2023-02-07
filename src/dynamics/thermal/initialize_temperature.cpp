/**
 * @file initialize_temperature.cpp
 * @brief Initialize function for temperature
 */

#include "initialize_temperature.hpp"

#include <interface/initialize/initialize_file_access.hpp>

#include <environment/global/simulation_time.hpp>
#include <string>

#include "initialize_node.hpp"

/* Import node properties and Cij/Rij Datas by reading CSV File (node.csv,
cij.csv, rij.csv) Detailed process of reading node properties from CSV File, and
CSV file formats of node properties is written in Init_Node.cpp

[File Formats of Node.csv]
column 1: Node_id(int)
column 2: Node_label(string)
column 3: capacity
column 4: alpha
column 5: area
column 6,7,8: normal vector of surface(body frame)
column 9: initial internal heat(W)
column 10: initial temperature(K)

First row is for Header, data begins from the second row

[File Formats of Cij.csv Rij.Csv]
 Cij information is written in Row i, column j
 Ex.
    0.5 0.3 0.2
    0.1 0.4 0.1
    0.2 0.2 0.5

    →R12 = 0.3
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

Temperature* InitTemperature(const std::string ini_path, const double rk_prop_step_sec) {
  auto mainIni = IniAccess(ini_path);

  vector<Node> vnodes;
  vector<vector<double>> cij;
  vector<vector<double>> rij;
  vector<vector<string>> vnodestr;  // string vector of node property data
  int nodes_num = 1;

  bool is_calc_enabled = mainIni.ReadEnable("THERMAL", "calculation");
  if (is_calc_enabled == false) {
    // Return here to avoid CSV file reading
    Temperature* temperature;
    temperature = new Temperature();
    return temperature;
  }

  // read ini-file settings
  string file_path = mainIni.ReadString("THERMAL", "thermal_file_directory");
  bool debug = mainIni.ReadBoolean("THERMAL", "debug");

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

  // Read Cij,Rij data from CSV File
  string filepath_cij = file_path + "cij.csv";
  string filepath_rij = file_path + "rij.csv";
  IniAccess conf_cij(filepath_cij);
  IniAccess conf_rij(filepath_rij);
  conf_cij.ReadCsvDouble(cij,
                         nodes_num + 1);  // for Cij,Rij, include outerspace as an additional node
  conf_rij.ReadCsvDouble(rij, nodes_num + 1);

  Temperature* temperature;
  temperature = new Temperature(cij, rij, vnodes, nodes_num, rk_prop_step_sec, is_calc_enabled, debug);
  return temperature;
}
