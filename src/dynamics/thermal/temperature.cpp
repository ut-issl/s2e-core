/**
 * @file temperature.hpp
 * @brief Initialize temperature
 */

#include "temperature.hpp"

#include <cmath>
#include <iostream>
#include <library/utilities/macros.hpp>
#include <vector>

using namespace std;

Temperature::Temperature(const vector<vector<double>> cij, const vector<vector<double>> rij, vector<Node> vnodes, const int node_num,
                         const double propstep, const bool is_calc_enabled, const bool debug)
    : cij_(cij),
      rij_(rij),
      vnodes_(vnodes),
      node_num_(node_num),
      prop_step_(propstep),  // ルンゲクッタ積分時間刻み幅
      is_calc_enabled_(is_calc_enabled),
      debug_(debug) {
  prop_time_ = 0;
  if (debug_) {
    PrintParams();
  }
}

Temperature::Temperature() {
  node_num_ = 0;
  prop_step_ = 0.0;
  is_calc_enabled_ = false;
  debug_ = false;
}

Temperature::~Temperature() {}

void Temperature::Propagate(libra::Vector<3> sun_direction, const double endtime) {
  if (!is_calc_enabled_) return;
  while (endtime - prop_time_ - prop_step_ > 1.0e-6) {
    RungeOneStep(prop_time_, prop_step_, sun_direction, node_num_);
    prop_time_ += prop_step_;
  }
  RungeOneStep(prop_time_, endtime - prop_time_, sun_direction, node_num_);
  prop_time_ = endtime;

  if (debug_) {
    cout << fixed;
    cout << "Time: " << endtime << "  Temp:  ";
    for (auto itr = vnodes_.begin(); itr != vnodes_.end(); ++itr) {
      cout << setprecision(4) << itr->GetTemperature_K() << "  ";
    }
    cout << "SolarR:  ";
    for (auto itr = vnodes_.begin(); itr != vnodes_.end(); ++itr) {
      cout << setprecision(4) << itr->GetSolarRadiation() << "  ";
    }
    cout << "SunDir:  ";
    double norm_sund = sun_direction.CalcNorm();
    for (int i = 0; i < 3; i++) {
      cout << setprecision(3) << sun_direction[i] / norm_sund << "  ";
    }
    cout << endl;
  }
}

void Temperature::RungeOneStep(double t, double dt, libra::Vector<3> sun_direction, int node_num) {
  vector<double> x(node_num);
  for (int i = 0; i < node_num; i++) {
    x[i] = vnodes_[i].GetTemperature_K();
  }

  vector<double> k1(node_num), k2(node_num), k3(node_num), k4(node_num);
  vector<double> xk2(node_num), xk3(node_num), xk4(node_num);

  k1 = OdeTemperature(x, t, sun_direction, node_num);
  for (int i = 0; i < node_num; i++) {
    xk2[i] = x[i] + (dt / 2.0) * k1[i];
  }

  k2 = OdeTemperature(xk2, (t + dt / 2.0), sun_direction, node_num);
  for (int i = 0; i < node_num; i++) {
    xk3[i] = x[i] + (dt / 2.0) * k2[i];
  }

  k3 = OdeTemperature(xk3, (t + dt / 2.0), sun_direction, node_num);
  for (int i = 0; i < node_num; i++) {
    xk4[i] = x[i] + dt * k3[i];
  }

  k4 = OdeTemperature(xk4, (t + dt), sun_direction, node_num);

  vector<double> next_x(node_num);  // temperature at next step
  for (int i = 0; i < node_num; i++) {
    next_x[i] = x[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }

  for (int i = 0; i < node_num; i++) {
    vnodes_[i].SetTemperature_K(next_x[i]);
  }
}

vector<double> Temperature::OdeTemperature(vector<double> x, double t, libra::Vector<3> sun_direction, int node_num) {
  // TODO: consider the following unused arguments are really needed
  UNUSED(x);
  UNUSED(t);

  vector<double> dTdt(node_num);
  for (int i = 0; i < node_num; i++) {
    double solar = vnodes_[i].CalcSolarRadiation(sun_direction);  // solar radiation[W]
    double internal = vnodes_[i].GetInternalHeat();               // internal(generated) heat[W]

    double coupling_heat = 0;      // Coupling of node i and j by heat transfer
    double radiation_heat = 0;     // Coupling of node i and j by thermal radiation
    double const sigma = 5.67E-8;  // Stefan-Boltzmann Constant
    for (int j = 0; j < node_num; j++) {
      coupling_heat += cij_[i][j] * (vnodes_[j].GetTemperature_K() - vnodes_[i].GetTemperature_K());
      radiation_heat += sigma * rij_[i][j] * (pow(vnodes_[j].GetTemperature_K(), 4) - pow(vnodes_[i].GetTemperature_K(), 4));
    }
    dTdt[i] = (coupling_heat + radiation_heat + solar + internal) / vnodes_[i].GetCapacity();
  }
  return dTdt;
}

void Temperature::AddHeaterPower(vector<double> heater_power) {
  for (auto itr = vnodes_.begin(); itr != vnodes_.end(); ++itr) {
    if ((itr->GetHeaterNodeId()) > 0) {
      itr->SetInternalHeat(heater_power[itr->GetHeaterNodeId()]);  // Set internal heat
    } else {
      itr->SetInternalHeat(0.0);  // Nodes without heater
    }
  }
}

vector<Node> Temperature::GetVnodes() const { return vnodes_; }

string Temperature::GetLogHeader() const {
  string str_tmp = "";
  for (int i = 0; i < node_num_; i++) {
    string str_node = "temp_" + to_string(vnodes_[i].GetNodeId()) + " (" + vnodes_[i].GetNodeLabel() + ")";
    str_tmp += WriteScalar(str_node, "deg");
  }
  /*
  下記のコードのようにiteratorでアクセスすると,
  インスタンス化されていないメンバ関数のアドレスを取っていることになり
  うまくいかない
  for (auto itr = vnodes_.cbegin() + 1; itr != vnodes_.cend(); ++itr) {
          string str_node = "temp_" + to_string((*itr->GetNodeId)()) + " (" +
  to_string((*itr->GetNodeLabel)()) + ")"; str_tmp += WriteScalar(str_node,
  "[deg]");
  }*/
  return str_tmp;
}

string Temperature::GetLogValue() const {
  string str_tmp = "";
  for (int i = 0; i < node_num_; i++) {
    str_tmp += WriteScalar(vnodes_[i].GetTemperature_deg());
  }
  return str_tmp;
}

void Temperature::PrintParams(void) {
  cout << "< Print Thermal Parameters >" << endl;
  cout << "IsCalcEnabled: " << is_calc_enabled_ << endl;
  cout << "Vnodes:" << endl;
  for (auto itr = vnodes_.begin(); itr != vnodes_.end(); ++itr) {
    itr->PrintParam();
  }
  cout << std::fixed;
  cout << "Cij:" << endl;
  for (int i = 0; i < (node_num_ + 1); i++) {
    for (int j = 0; j < (node_num_ + 1); j++) {
      cout << std::setprecision(4) << cij_[i][j] << "  ";
    }
    cout << endl;
  }
  cout << "Rij:" << endl;
  for (int i = 0; i < (node_num_ + 1); i++) {
    for (int j = 0; j < (node_num_ + 1); j++) {
      cout << std::setprecision(4) << rij_[i][j] << "  ";
    }
    cout << endl;
  }
  cout << "**************************************" << endl;
}
