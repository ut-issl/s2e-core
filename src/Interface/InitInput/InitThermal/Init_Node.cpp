#include "../Initialize.h"
#include <iostream>
#include <math.h>
#include <string>
#include <typeinfo>
#include <vector>
using namespace std;

#include "../../../Dynamics/Thermal/Node.h"

/* Import node properties and Cij/Rij Datas by reading CSV File (Node.csv,
Cij.csv, Rij.csv) Detailed process of reading node properties from CSV File, and
CSV file formats of node properties is written in Init_Node.cpp

[File Formats of Node.csv]
column 1: Node_id(int)
column 2: Node_label(string)
column 3: capacity
column 4: alpha
column 5: area
column 6,7,8: normal vector of surface(body frame)
column 9: initial internal heat(J)
column 10: initial temperature(K)

First row is for Header, data begins from the second row
Ex.
Node_id,Node_label,capacity,solar_radiation,internal_heat,temperature
1,BUS,2.5,0,0,293
2,SAP,3.2,10,30,288
*/

Node InitNode(const vector<string> &node_str) {
  int node_id = 0;            // node number
  string node_label = "temp"; // node name
  int heater_node_id = 0;     // heater node index
  double temperature = 0;     // [K]
  double capacity = 0;        // [J/K]
  double internal_heat = 0;   // generated heat[J]
  double alpha = 0;           //[m^2]
  double area = 0;

  node_id = stoi(node_str[0]);        // column 1
  node_label = node_str[1];           // column 2
  heater_node_id = stoi(node_str[2]); // column 3
  capacity = stod(node_str[3]);       // column 4
  alpha = stod(node_str[4]);          // column 5
  area = stod(node_str[5]);           // column 6
  Vector<3> normal_v_b;               // column 7-9
  for (int i = 0; i < 3; i++) {
    normal_v_b[i] = stod(node_str[6 + i]);
  }                                  // body frame
  internal_heat = stod(node_str[9]); // column 10
  temperature = stod(node_str[10]);  // column 11

  Node node(node_id, node_label, heater_node_id, temperature, capacity,
            internal_heat, alpha, area, normal_v_b);
  return node;
}
