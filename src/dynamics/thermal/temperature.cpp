/**
 * @file temperature.hpp
 * @brief Initialize temperature
 */

#include "temperature.hpp"

#include <cmath>
#include <environment/global/physical_constants.hpp>
#include <iostream>
#include <library/utilities/macros.hpp>
#include <vector>

using namespace std;

Temperature::Temperature(const vector<vector<double>> conductance_matrix_W_K, const vector<vector<double>> radiation_matrix_m2, vector<Node> nodes,
                         vector<Heatload> heatloads, vector<Heater> heaters, vector<HeaterController> heater_controllers, const int node_num,
                         const double propagation_step_s, const bool is_calc_enabled, const SolarCalcSetting solar_calc_setting, const bool debug)
    : conductance_matrix_W_K_(conductance_matrix_W_K),
      radiation_matrix_m2_(radiation_matrix_m2),
      nodes_(nodes),
      heatloads_(heatloads),
      heaters_(heaters),
      heater_controllers_(heater_controllers),
      node_num_(node_num),
      propagation_step_s_(propagation_step_s),  // ルンゲクッタ積分時間刻み幅
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

void Temperature::Propagate(libra::Vector<3> sun_direction_b, const double time_end_s) {
  if (!is_calc_enabled_) return;
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
      cout << setprecision(4) << itr->GetTemperature_K() << "  ";
    }
    cout << "SolarR:  ";
    for (auto itr = nodes_.begin(); itr != nodes_.end(); ++itr) {
      cout << setprecision(4) << itr->GetSolarRadiation_W() << "  ";
    }
    cout << "SunDir:  ";
    double norm_sun_direction_b = sun_direction_b.CalcNorm();
    for (int i = 0; i < 3; i++) {
      cout << setprecision(3) << sun_direction_b[i] / norm_sun_direction_b << "  ";
    }
    cout << "Heatload:  ";
    for (auto itr = heatloads_.begin(); itr != heatloads_.end(); ++itr) {
      cout << setprecision(3) << itr->GetTotalHeatload_W() << "  ";
    }
    cout << endl;
  }
}

void Temperature::CalcRungeOneStep(double time_now_s, double time_step_s, libra::Vector<3> sun_direction_b, int node_num) {
  vector<double> temperatures_now_K(node_num);
  for (int i = 0; i < node_num; i++) {
    temperatures_now_K[i] = nodes_[i].GetTemperature_K();
  }

  vector<double> k1(node_num), k2(node_num), k3(node_num), k4(node_num);
  vector<double> xk2(node_num), xk3(node_num), xk4(node_num);

  k1 = CalcTemperatureDifferentials(temperatures_now_K, time_now_s, sun_direction_b, node_num);
  for (int i = 0; i < node_num; i++) {
    xk2[i] = temperatures_now_K[i] + (time_step_s / 2.0) * k1[i];
  }

  k2 = CalcTemperatureDifferentials(xk2, (time_now_s + time_step_s / 2.0), sun_direction_b, node_num);
  for (int i = 0; i < node_num; i++) {
    xk3[i] = temperatures_now_K[i] + (time_step_s / 2.0) * k2[i];
  }

  k3 = CalcTemperatureDifferentials(xk3, (time_now_s + time_step_s / 2.0), sun_direction_b, node_num);
  for (int i = 0; i < node_num; i++) {
    xk4[i] = temperatures_now_K[i] + time_step_s * k3[i];
  }

  k4 = CalcTemperatureDifferentials(xk4, (time_now_s + time_step_s), sun_direction_b, node_num);

  vector<double> temperatures_next_K(node_num);
  for (int i = 0; i < node_num; i++) {
    temperatures_next_K[i] = temperatures_now_K[i] + (time_step_s / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }

  for (int i = 0; i < node_num; i++) {
    nodes_[i].SetTemperature_K(temperatures_next_K[i]);
  }
}

vector<double> Temperature::CalcTemperatureDifferentials(vector<double> temperatures_K, double t, libra::Vector<3> sun_direction, int node_num) {
  // TODO: consider the following unused arguments are really needed
  UNUSED(temperatures_K);

  vector<double> differentials_K_s(node_num);
  for (int i = 0; i < node_num; i++) {
    heatloads_[i].SetElapsedTime_s(t);
    if (nodes_[i].GetNodeType() == NodeType::kDiffusive) {
      if (solar_calc_setting_ == SolarCalcSetting::kEnable) {
        double solar_radiation_W = nodes_[i].CalcSolarRadiation_W(sun_direction);
        heatloads_[i].SetSolarHeatload_W(solar_radiation_W);
      }
      double heater_power_W = GetHeaterPower_W(i);
      heatloads_[i].SetHeaterHeatload_W(heater_power_W);
      heatloads_[i].CalcInternalHeatload();
      heatloads_[i].UpdateTotalHeatload();
      double total_heatload_W = heatloads_[i].GetTotalHeatload_W();  // Total heatload (solar + internal + heater)[W]

      double conductive_heat_input_W = 0;
      double radiative_heat_input_W = 0;
      for (int j = 0; j < node_num; j++) {
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

double Temperature::GetHeaterPower_W(int node_id) {
  int heater_id = nodes_[node_id].GetHeaterId();
  double heater_power_W = 0.0;
  if (heater_id > 0) {
    heater_power_W = heaters_[heater_id - 1].GetPowerOutput_W();
  }
  return heater_power_W;
}

void Temperature::UpdateHeaterStatus(void) {
  // [FIXME] Heater status doesn't get updated...
  for (auto itr = nodes_.begin(); itr != nodes_.end(); ++itr) {
    int heater_id = itr->GetHeaterId();
    if (heater_id > 0) {
      double temperature_degC = itr->GetTemperature_degC();
      Heater* p_heater = &heaters_[heater_id - 1];
      heater_controllers_[heater_id - 1].ControlHeater(p_heater, temperature_degC);
    }
  }
}

string Temperature::GetLogHeader() const {
  string str_tmp = "";
  for (int i = 0; i < node_num_; i++) {
    // Do not retrieve boundary node values
    if (nodes_[i].GetNodeType() != NodeType::kBoundary) {
      string str_node = "temp_" + to_string(nodes_[i].GetNodeId()) + " (" + nodes_[i].GetNodeName() + ")";
      str_tmp += WriteScalar(str_node, "deg");
    }
  }
  for (int i = 0; i < node_num_; i++) {
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
  for (int i = 0; i < node_num_; i++) {
    // Do not retrieve boundary node values
    if (nodes_[i].GetNodeType() != NodeType::kBoundary) {
      str_tmp += WriteScalar(nodes_[i].GetTemperature_degC());
    }
  }
  for (int i = 0; i < node_num_; i++) {
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
  cout << "Vnodes:" << endl;
  for (auto itr = nodes_.begin(); itr != nodes_.end(); ++itr) {
    itr->PrintParam();
  }
  cout << "Vheaters:" << endl;
  for (auto itr = heaters_.begin(); itr != heaters_.end(); ++itr) {
    itr->PrintParam();
  }
  cout << std::fixed;
  cout << "Cij:" << endl;
  for (int i = 0; i < (node_num_); i++) {
    for (int j = 0; j < (node_num_); j++) {
      cout << std::setprecision(4) << conductance_matrix_W_K_[i][j] << "  ";
    }
    cout << endl;
  }
  cout << "Rij:" << endl;
  for (int i = 0; i < (node_num_); i++) {
    for (int j = 0; j < (node_num_); j++) {
      cout << std::setprecision(4) << radiation_matrix_m2_[i][j] << "  ";
    }
    cout << endl;
  }
  cout << "**************************************" << endl;
}
