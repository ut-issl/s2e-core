#include <iostream>
#include <vector>
#include <sstream>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <cmath>
#include "GnssSatellites.h"
#include "../../Interface/LogOutput/LogUtility.h"
#include "../../Library/sgp4/sgp4unit.h" //gstimeが欲しい
#include "../../Library/sgp4/sgp4ext.h" //jdayが欲しい

using namespace std;

GnssSatellites::GnssSatellites(int satellite_num, int time_stamp_num, int num_of_days) : num_of_satellites_(satellite_num), num_of_time_stamps_(time_stamp_num), num_of_days_(num_of_days), half_points((num_of_points - 1) / 2), gnss_true()
{
}

//デコンストラクタ 特に無し(ポインタは存在しないので)
GnssSatellites::~GnssSatellites(){}

void GnssSatellites::ReadSP3(vector<vector<string>>& sp3_file)
{
  if (!IsCalcEnabled) return;

  //libra::Vector<3>に関する多次元配列になっているので、GNSS衛星の数だけ拡張する。
  gnss_satellites_position_table_ecef_.resize(num_of_satellites_);
  gnss_satellites_position_table_eci_.resize(num_of_satellites_);

  for(int day = 0;day < num_of_days_;++day){
    for(int i = 0;i < num_of_time_stamps_*(num_of_satellites_ + 1);++i){
      string str = sp3_file.at(day).at(i);
      int id = i%(num_of_satellites_ + 1) - 1; //衛星の番号(ファイルの間は固定だとする)
      istringstream iss{str};
      vector<string> s;
      if(i%(num_of_satellites_ + 1) == 0){
        //時刻を抜き出す
        for(int j = 0;j < 7;++j){
          string tmp;
          iss >> tmp;
          s.push_back(tmp);
        }
        int year = stoi(s.at(1));
        int month = stoi(s.at(2));
        int jd_day = stoi(s.at(3));
        int hr = stoi(s.at(4));
        int minute = stoi(s.at(5));
        double sec = stod(s.at(6));
        double jd;
        jday(year, month, jd_day, hr, minute, sec, jd);
        side_table_.push_back(gstime(jd));
      }else{
        //x,y,zの座標を抜き出す
        for(int j = 0;j < 9;++j){
          string tmp;
          iss >> tmp;
          s.push_back(tmp);
        }
        libra::Vector<3> position;
        for(int j = 0;j < 3;++j){
          position[j] = stod(s.at(j+1));
        }

        // [km] -> [m]
        position *= 1000.0;

        gnss_satellites_position_table_ecef_.at(id).push_back(position);
      }
    }
  }

  //雑なecef to eciの変換
  for(int id = 0;id < num_of_satellites_;++id){
    for(int i = 0;i < gnss_satellites_position_table_ecef_.at(id).size();++i){
      double x = gnss_satellites_position_table_ecef_.at(id).at(i)[0];
      double y = gnss_satellites_position_table_ecef_.at(id).at(i)[1];
      double z = gnss_satellites_position_table_ecef_.at(id).at(i)[2];

      double c = cos(side_table_.at(i));
      double s = sin(side_table_.at(i));

      libra::Vector<3> position;
      position[0] = c*x - s*y;
      position[1] = s*x + c*y;
      position[2] = z;

      gnss_satellites_position_table_eci_.at(id).push_back(position);
    }
  }

  //衛星のサイズ分拡張
  gnss_satellites_position_ecef_.resize(num_of_satellites_);
  gnss_satellites_position_eci_.resize(num_of_satellites_);

  //update用に時間の逆引き用のTableを作る
  time_table_.resize(num_of_days_*num_of_time_stamps_);
  const double seconds_of_one_day = 24.0*60.0*60.0;
  const double time_interval = seconds_of_one_day/(double)num_of_time_stamps_;
  for(int i = 0;i < num_of_days_*num_of_time_stamps_;++i){
    //Tableは-1日目から開始
    time_table_.at(i) = (double)(i - num_of_time_stamps_)*time_interval;
  }

  //clock 初期化
  gnss_satellites_clock.assign(num_of_satellites_, 0.0);
}

//補間のためのデータを作る
void GnssSatellites::InitTable(int start_hr, int start_min, double start_sec, double elapsed_time_sec)
{
  if (!IsCalcEnabled) return;

  double hr = start_hr;
  double min = start_min;
  double sec = start_sec;
  const double now_sec = hr*60.0*60.0 + min*60.0 + sec + elapsed_time_sec;
  
  int index = upper_bound(time_table_.begin(), time_table_.end(), now_sec) - time_table_.begin();
  //一番近い点を中心の起点にする
  if(abs(now_sec - time_table_.at(index - 1)) < abs(time_table_.at(index) - now_sec)) --index;

  pre_index = index;

  center_time = time_table_.at(index);
  eci_x.resize(num_of_satellites_);
  eci_y.resize(num_of_satellites_);
  eci_z.resize(num_of_satellites_);
  ecef_x.resize(num_of_satellites_);
  ecef_y.resize(num_of_satellites_);
  ecef_z.resize(num_of_satellites_);

  for(int i = -half_points;i <= half_points;++i) time_period.push_back(time_table_.at(index + i) - center_time);

  for(int sat_id = 0;sat_id < num_of_satellites_;++sat_id){
    for(int i = -half_points;i <= half_points;++i){
      eci_x.at(sat_id).push_back(gnss_satellites_position_table_eci_.at(sat_id).at(index + i)[0]);
      eci_y.at(sat_id).push_back(gnss_satellites_position_table_eci_.at(sat_id).at(index + i)[1]);
      eci_z.at(sat_id).push_back(gnss_satellites_position_table_eci_.at(sat_id).at(index + i)[2]);

      ecef_x.at(sat_id).push_back(gnss_satellites_position_table_ecef_.at(sat_id).at(index + i)[0]);
      ecef_y.at(sat_id).push_back(gnss_satellites_position_table_ecef_.at(sat_id).at(index + i)[1]);
      ecef_z.at(sat_id).push_back(gnss_satellites_position_table_ecef_.at(sat_id).at(index + i)[2]);
    }

    double time = now_sec - center_time;

    gnss_satellites_position_eci_.at(sat_id)[0] = TrigonometricInterpolation(time_period, eci_x.at(sat_id), time);
    gnss_satellites_position_eci_.at(sat_id)[1] = TrigonometricInterpolation(time_period, eci_y.at(sat_id), time);
    gnss_satellites_position_eci_.at(sat_id)[2] = TrigonometricInterpolation(time_period, eci_z.at(sat_id), time);

    gnss_satellites_position_ecef_.at(sat_id)[0] = TrigonometricInterpolation(time_period, ecef_x.at(sat_id), time);
    gnss_satellites_position_ecef_.at(sat_id)[1] = TrigonometricInterpolation(time_period, ecef_y.at(sat_id), time);
    gnss_satellites_position_ecef_.at(sat_id)[2] = TrigonometricInterpolation(time_period, ecef_z.at(sat_id), time);
  }

  return;
}

void GnssSatellites::Update(int start_hr, int start_min, double start_sec, double elapsed_time_sec)
{
  if (!IsCalcEnabled) return;
  
  double hr = start_hr;
  double min = start_min;
  double sec = start_sec;
  const double now_sec = hr*60.0*60.0 + min*60.0 + sec + elapsed_time_sec;

  UpdatePosition(now_sec);
  UpdateClock(now_sec);

  return;
}

/*
内挿法としては、Trigonometric interpolation 
https://en.wikipedia.org/wiki/Trigonometric_interpolation#
http://acc.igs.org/orbits/orbit-interp_gpssoln03.pdf
*/
void GnssSatellites::UpdatePosition(double now_sec)
{
  int index = pre_index;

  //一番近い点を中心の起点にする
  if(abs(time_table_.at(index + 1) - now_sec) < abs(time_table_.at(index) - now_sec)){
    ++index;

    time_period.clear();
    for(int i = -half_points;i <= half_points;++i){
      time_period.push_back(time_table_.at(index + i) - center_time);
    }

    for(int sat_id = 0;sat_id < num_of_satellites_;++sat_id){
      eci_x.at(sat_id).clear();
      eci_y.at(sat_id).clear();
      eci_z.at(sat_id).clear();

      ecef_x.at(sat_id).clear();
      ecef_y.at(sat_id).clear();
      ecef_z.at(sat_id).clear();

      for(int i = -half_points;i <= half_points;++i){
        eci_x.at(sat_id).push_back(gnss_satellites_position_table_eci_.at(sat_id).at(index + i)[0]);
        eci_y.at(sat_id).push_back(gnss_satellites_position_table_eci_.at(sat_id).at(index + i)[1]);
        eci_z.at(sat_id).push_back(gnss_satellites_position_table_eci_.at(sat_id).at(index + i)[2]);

        ecef_x.at(sat_id).push_back(gnss_satellites_position_table_ecef_.at(sat_id).at(index + i)[0]);
        ecef_y.at(sat_id).push_back(gnss_satellites_position_table_ecef_.at(sat_id).at(index + i)[1]);
        ecef_z.at(sat_id).push_back(gnss_satellites_position_table_ecef_.at(sat_id).at(index + i)[2]);
      }
    }

    center_time = time_table_.at(index);
    pre_index = index;
  }
  
  double time = now_sec - center_time;

  for(int sat_id = 0;sat_id < num_of_satellites_;++sat_id){
    gnss_satellites_position_eci_.at(sat_id)[0] = TrigonometricInterpolation(time_period, eci_x.at(sat_id), time);
    gnss_satellites_position_eci_.at(sat_id)[1] = TrigonometricInterpolation(time_period, eci_y.at(sat_id), time);
    gnss_satellites_position_eci_.at(sat_id)[2] = TrigonometricInterpolation(time_period, eci_z.at(sat_id), time);

    gnss_satellites_position_ecef_.at(sat_id)[0] = TrigonometricInterpolation(time_period, ecef_x.at(sat_id), time);
    gnss_satellites_position_ecef_.at(sat_id)[1] = TrigonometricInterpolation(time_period, ecef_y.at(sat_id), time);
    gnss_satellites_position_ecef_.at(sat_id)[2] = TrigonometricInterpolation(time_period, ecef_z.at(sat_id), time);
  }

  gnss_true.Update(gnss_satellites_position_eci_);

  return;
}

// 仮仕様
void GnssSatellites::UpdateClock(double now_sec)
{
  for(int sat_id = 0;sat_id < num_of_satellites_;++sat_id) gnss_satellites_clock.at(sat_id) += 0.0;
}

int GnssSatellites::GetNumOfSatellites() const
{
  return num_of_satellites_;
}

bool GnssSatellites::GetWhetherValid(int sat_id) const
{
  if(sat_id >= GetNumOfSatellites()) return false;
  else return true;
}

const GnssSatellites_Info& GnssSatellites::Get_true_info() const
{
  return gnss_true;
}

libra::Vector<3> GnssSatellites::GetSatellitePositionEcef(const int sat_id) const
{
  //sat_id is wrong
  if(sat_id >= num_of_satellites_){
    libra::Vector<3> res(0);
    return res;
  }

  return gnss_satellites_position_ecef_.at(sat_id);
}
libra::Vector<3> GnssSatellites::GetSatellitePositionEci(const int sat_id) const
{
  //sat_id is wrong
  if(sat_id >= num_of_satellites_){
    libra::Vector<3> res(0);
    return res;
  }

  return gnss_satellites_position_eci_.at(sat_id);
}

double GnssSatellites::GetSatelliteClock(const int sat_id) const
{
  if(sat_id >= num_of_satellites_) return 0.0;
  else return gnss_satellites_clock.at(sat_id);
}

double GnssSatellites::GetPseudoRangeECEF(const int sat_id, libra::Vector<3> sat_position, double sat_clock, const double frequency) const
{
  //sat_id is wrong
  if(sat_id >= num_of_satellites_) return 0.0;

  double res = 0.0;
  for(int i = 0;i < 3;++i){
    res += (sat_position[i] - gnss_satellites_position_ecef_.at(sat_id)[i])*(sat_position[i] - gnss_satellites_position_ecef_.at(sat_id)[i]);
  }
  res = sqrt(res);

  // clock bias
  res += sat_clock - gnss_satellites_clock.at(sat_id);

  //ionospheric delay
  const double ionospheric_delay = AddIonosphericDelay(sat_id, sat_position, frequency, ECEF);
  
  res += ionospheric_delay;

  return res;
}

double GnssSatellites::GetPseudoRangeECI(const int sat_id, libra::Vector<3> sat_position, double sat_clock, const double frequency) const
{
  //sat_id is wrong
  if(sat_id >= num_of_satellites_) return 0.0;

  double res = 0.0;
  for(int i = 0;i < 3;++i){
    res += (sat_position[i] - gnss_satellites_position_eci_.at(sat_id)[i])*(sat_position[i] - gnss_satellites_position_eci_.at(sat_id)[i]);
  }
  res = sqrt(res);

  //clock bias
  res += sat_clock - gnss_satellites_clock.at(sat_id);

  //ionospheric delay
  const double ionospheric_delay = AddIonosphericDelay(sat_id, sat_position, frequency, ECI);
  
  res += ionospheric_delay;

  return res;
}

pair<double, double> GnssSatellites::GetCarrierPhaseECEF(const int sat_id, libra::Vector<3> sat_position, double sat_clock, const double frequency) const
{
  //sat_id is wrong
  if(sat_id >= num_of_satellites_) return {0.0, 0.0};

  double res = 0.0;
  for(int i = 0;i < 3;++i){
    res += (sat_position[i] - gnss_satellites_position_ecef_.at(sat_id)[i])*(sat_position[i] - gnss_satellites_position_ecef_.at(sat_id)[i]);
  }
  res = sqrt(res);

  // clock bias
  res += sat_clock - gnss_satellites_clock.at(sat_id);

  //ionospheric delay
  const double ionospheric_delay = AddIonosphericDelay(sat_id, sat_position, frequency, ECEF);
  
  res -= ionospheric_delay;
  
  // wavelength
  double lambda = speed_of_light*1e-6/frequency;
  double cycle = res/lambda;

  double bias = floor(cycle);
  cycle -= bias;

  return {cycle, bias};
}

pair<double, double> GnssSatellites::GetCarrierPhaseECI(const int sat_id, libra::Vector<3> sat_position, double sat_clock, const double frequency) const
{
  //sat_id is wrong
  if(sat_id >= num_of_satellites_) return {0.0, 0.0};

  double res = 0.0;
  for(int i = 0;i < 3;++i){
    res += (sat_position[i] - gnss_satellites_position_eci_.at(sat_id)[i])*(sat_position[i] - gnss_satellites_position_eci_.at(sat_id)[i]);
  }
  res = sqrt(res);

  //clock bias
  res += sat_clock - gnss_satellites_clock.at(sat_id);

  //ionospheric delay
  const double ionospheric_delay = AddIonosphericDelay(sat_id, sat_position, frequency, ECI);
  
  res -= ionospheric_delay;

  // wavelength
  double lambda = speed_of_light*1e-6/frequency;
  double cycle = res/lambda;

  double bias = floor(cycle);
  cycle -= bias;

  return {cycle, bias};
}

//ここらへん結構数値誤差結構気にしたいのにすごい雑な計算で嫌だなぁ...
double GnssSatellites::TrigonometricInterpolation(vector<double> time_period, vector<double> position, double time)
{
  int n = time_period.size();
  double w = 2.0*M_PI/(24.0*60.0*60.0)*1.03; //適当に1日の長さで重みづけ
  double res = 0.0;

  for(int i = 0;i < n;++i){
    double t_k = 1.0;
    for(int j = 0;j < n;++j){
      if(i == j) continue;
      t_k *= sin(w*(time - time_period.at(j))/2.0)/sin(w*(time_period.at(i) - time_period.at(j))/2.0);
    }
    res += t_k*position.at(i);
  }

  return res;
}

// for Ionospheric delay I[m]
double GnssSatellites::AddIonosphericDelay(const int sat_id, const libra::Vector<3> sat_position, const double frequency, const bool flag) const
{
  //gnss_satが存在しない
  if(sat_id >= num_of_satellites_) return 0.0;

  const double Earth_hemisphere = 6378.1; //[km]

  double altitude = 0.0;
  for(int i = 0;i < 3;++i) altitude += sat_position[i]*sat_position[i];
  altitude = sqrt(altitude);
  altitude = altitude/1000.0 - Earth_hemisphere; //[m -> km]
  if(altitude >= 1000.0) return 0.0; //上空1000km以上に電離層は存在しない

  libra::Vector<3> gnss_position;
  if(flag == ECEF) gnss_position = GetSatellitePositionEcef(sat_id);
  else if(flag == ECI) gnss_position = GetSatellitePositionEci(sat_id);  

  double angle_rad = angle(sat_position, gnss_position - sat_position);
  const double defalut_delay = 20.0; //[m] デフォルトで用意しておく
  double delay = defalut_delay*(1000.0 - altitude)/1000.0/cos(angle_rad); //最大長を1000.0として補正する 傾きがあると長くなるのでcosで割る
  const double defalut_frequency = 1500.0; //[MHz]
  // 電離層遅延は周波数の2乗に反比例する
  delay *= defalut_frequency*defalut_frequency/(frequency*frequency);

  return delay;
}

string GnssSatellites::GetLogHeader() const
{
  string str_tmp = "";

  return str_tmp;
}

string GnssSatellites::GetLogValue() const
{
  string str_tmp = "";
  
  return str_tmp;
}