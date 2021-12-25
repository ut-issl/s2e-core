#pragma once

#ifndef __temperature_H__
#define __temperature_H__

#include <string>
#include <vector>
using namespace std;

#include "../../Interface/LogOutput/ILoggable.h"
#include "Node.h"

class Temperature: public ILoggable
{
protected:
  vector<vector<double>> cij_; //Coupling of node i and node j by heat conduction
  vector<vector<double>> rij_; //Coupling of node i and node j by thermal radiation
  vector<Node> vnodes_; //vector of nodes
  int node_num_; //number of nodes 
  double prop_step_; //積分刻み幅[sec]
  double prop_time_; //Temperatureクラス内での累積積分時間(end_timeに等しくなるまで積分する)
  bool isCalcEnabled_; //温度更新をするかどうかのブーリアン
  bool debug_;

  void RungeOneStep(double t, double dt, Vector<3> sun_direction, int node_num); // ルンゲクッタOne Step
  vector<double> OdeTemperature(vector<double> x, double t, const Vector<3> sun_direction, int node_num);   // 温度に関する常微分方程式, xはnodeの温度をならべたもの

public:
	Temperature(const vector<vector<double>> cij_,
		const vector<vector<double>> rij,
		vector<Node> vnodes,
		const int node_num,
		const double propstep,
		const bool is_calc_enabled,
		const bool debug);
	~Temperature();
	void Propagate(Vector<3> sun_direction,const double endtime); //太陽入熱量計算のため, 太陽方向の情報を入手
	vector<Node> GetVnodes() const;
	void AddHeaterPower(vector<double> heater_power);
	string GetLogHeader() const;
	string GetLogValue() const;
	void PrintParams(void);  //デバッグ出力
};
#endif //__temperature_H__
