#include <iostream>
#include <vector>
#include <sstream>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <cmath>
#include "GnssSatellites.h"
#include "../../Interface/LogOutput/LogUtility.h"
#include "../../Library/sgp4/sgp4unit.h" //for gstime() 
#include "../../Library/sgp4/sgp4ext.h" //for jday()

using namespace std;

tm* initilized_tm()
{
  tm* time_tm = (tm*)malloc(sizeof(tm));

  time_tm->tm_year = 0;
  time_tm->tm_mon = 0;
  time_tm->tm_mday = 0;
  time_tm->tm_hour = 0;
  time_tm->tm_min = 0;
  time_tm->tm_sec = 0;

  time_tm->tm_isdst = 0;
  time_tm->tm_yday = 0;
  time_tm->tm_wday = 0;
  
  #ifndef WIN32
    time_tm->tm_zone = NULL;
    time_tm->tm_gmtoff = 0;
  #endif

  return time_tm;
}

template<size_t N>
Vector<N> GnssSat_coordinate::TrigonometricInterpolation(const vector<double>& time_vector, const vector<Vector<N>>& values, double time) const
{
  int n = time_vector.size();
  double w = 2.0*M_PI/(24.0*60.0*60.0)*1.03; //coefficient of a day long
  Vector<N> res(0.0);

  for(int i = 0;i < n;++i){
    double t_k = 1.0;
    for(int j = 0;j < n;++j){
      if(i == j) continue;
      t_k *= sin(w*(time - time_vector.at(j))/2.0)/sin(w*(time_vector.at(i) - time_vector.at(j))/2.0);
    }
    for(int j = 0;j < N;++j){
      res(j) += t_k*values.at(i)(j);
    }
  }

  return res;
}

double GnssSat_coordinate::TrigonometricInterpolation(const vector<double>& time_vector, const vector<double>& values, double time) const
{
  int n = time_vector.size();
  double w = 2.0*M_PI/(24.0*60.0*60.0)*1.03; //coefficient of a day long
  double res = 0.0;

  for(int i = 0;i < n;++i){
    double t_k = 1.0;
    for(int j = 0;j < n;++j){
      if(i == j) continue;
      t_k *= sin(w*(time - time_vector.at(j))/2.0)/sin(w*(time_vector.at(i) - time_vector.at(j))/2.0);
    }
    res += t_k*values.at(i);
  }

  return res;
}

template<size_t N>
Vector<N> GnssSat_coordinate::LagrangeInterpolation(const vector<double>& time_vector, const vector<Vector<N>>& values, double time) const
{
  int n = time_vector.size();
  Vector<N> res(0.0);

  for(int i = 0;i < n;++i){
    double l_i = 1.0;
    for(int j = 0;j < n;++j){
      if(i == j) continue;
      l_i *= (time - time_vector.at(j))/(time_vector.at(i) - time_vector.at(j));
    }
    for(int j = 0;j < n;++j){
      res(j) += l_i*values.at(i)(j);
    }
  }

  return res;
}

double GnssSat_coordinate::LagrangeInterpolation(const vector<double>& time_vector, const vector<double>& values, double time) const
{
  int n = time_vector.size();
  double res = 0.0;
  for(int i = 0;i < n;++i){
    double l_i = 1.0;
    for(int j = 0;j < n;++j){
      if(i == j) continue;
      l_i *= (time - time_vector.at(j))/(time_vector.at(i) - time_vector.at(j));
    }
    res += values.at(i)*l_i;
  }

  return res;
}

int GnssSat_coordinate::GetIndexFromID(string sat_num) const
{
  if(sat_num.front() == 'P'){
    switch (sat_num.at(1))
    {
      case 'G':
        return stoi(sat_num.substr(2)) + gps_index_bias_;
      case 'R': 
        return stoi(sat_num.substr(2)) + glonass_index_bias_;
      case 'E':
        return stoi(sat_num.substr(2)) + galileo_index_bias_;
      case 'C':
        return stoi(sat_num.substr(2)) + beidou_index_bias_;
      case 'J':
        return stoi(sat_num.substr(2)) + qzss_index_bias_;
      default:
        return INT32_MAX;
        break;
    }
  }else{
    switch (sat_num.front())
    {
      case 'G':
        return stoi(sat_num.substr(1)) + gps_index_bias_;
      case 'R': 
        return stoi(sat_num.substr(1)) + glonass_index_bias_;
      case 'E':
        return stoi(sat_num.substr(1)) + galileo_index_bias_;
      case 'C':
        return stoi(sat_num.substr(1)) + beidou_index_bias_;
      case 'J':
        return stoi(sat_num.substr(1)) + qzss_index_bias_;
      default:
        return INT32_MAX;
        break;
    }
  }
}

string GnssSat_coordinate::GetIDFromIndex(int index) const
{
  string res;
  if(index < glonass_index_bias_){
    res = 'G';
    if(index - gps_index_bias_ < 10) res += '0';
    res += to_string(index - gps_index_bias_);
  }else if(index < galileo_index_bias_){
    res = 'R';
    if(index - glonass_index_bias_ < 10) res += '0';
    res += to_string(index - glonass_index_bias_);
  }else if(index < beidou_index_bias_){
    res = 'E';
    if(index - galileo_index_bias_ < 10) res += '0';
    res += to_string(index - galileo_index_bias_);
  }else if(index < qzss_index_bias_){
    res = 'C';
    if(index - beidou_index_bias_ < 10) res += '0';
    res += to_string(index - beidou_index_bias_);
  }else{
    res = 'J';
    if(index - qzss_index_bias_ < 10) res += '0';
    res += to_string(index - qzss_index_bias_);
  }

  return res;
}

int GnssSat_coordinate::GetNumOfSatellites() const
{
  return all_sat_num_;
}

bool GnssSat_coordinate::GetWhetherValid(int sat_id) const
{
  if(sat_id >= all_sat_num_) return false;
  return validate_.at(sat_id);
}

void GnssSat_coordinate::ProcessData(vector<vector<string>>& raw_file, vector<pair<string, vector<string>>>& data)
{
  for(int i = 0;i < raw_file.size();++i){
    int num_of_time_stamps = 0;
    int num_of_sat = 0;

    for(int j = 0;j < 3;++j){
      if(j == 0){
        istringstream iss{raw_file.at(i).at(j)};
        for(int k = 0;k < 7;++k){
          //how many time stamps are written?
          string each;
          iss >> each;
          if(k == 6){
            num_of_time_stamps = stoi(each);
            if(num_of_time_stamps%24) num_of_time_stamps -= num_of_time_stamps%24;
          }
        }
      }else if(j == 2){
        istringstream iss{raw_file.at(i).at(j)};
        for(int k = 0;k < 2;++k){
          string each;
          iss >> each;
          if(k == 1){
            //how many satellites
            num_of_sat = stoi(each);
          }
        }
      }
    }

    int j = 3;
    while(raw_file.at(i).at(j).front() != '*') ++j;
    int initial_row = j;
    for(j = 0;j < (num_of_sat + 1)*num_of_time_stamps;++j){
      int index = j + initial_row;
      if(j%(num_of_sat + 1) == 0) data.push_back({raw_file.at(i).at(index), vector<string>()});
      else data.back().second.push_back(raw_file.at(i).at(index));
    }
  }
}

void GnssSat_coordinate::ProcessData(vector<vector<string>>& raw_file, vector<pair<string, vector<string>>>& data, bool ur_flag)
{
  for(int i = 0;i < raw_file.size();++i){
    int num_of_sat = 0;
    int num_of_time_stamps_in_file = 0;

    for(int j = 0;j < 3;++j){
      if(j == 0){
        istringstream iss{raw_file.at(i).at(j)};
        for(int k = 0;k < 7;++k){
          //How many time stamps are written?
          string each;
          iss >> each;
          if(k == 6){
            num_of_time_stamps_in_file = stoi(each);
            if(num_of_time_stamps_in_file%24) num_of_time_stamps_in_file -= num_of_time_stamps_in_file%24;
          }
        }
      }else if(j == 2){
        istringstream iss{raw_file.at(i).at(j)};
        for(int k = 0;k < 2;++k){
          string each;
          iss >> each;
          if(k == 1){
            //how many satellites
            num_of_sat = stoi(each);
          }
        }
      }
    }

    vector<string> tmp_data;
    int j = 3;
    while(raw_file.at(i).at(j).front() != '*') ++j;
    int initial_row = j;
    for(j = initial_row;j < initial_row + (num_of_sat + 1)*num_of_time_stamps_in_file;++j) tmp_data.push_back(raw_file.at(i).at(j));

    int n = (num_of_sat + 1)*num_of_time_stamps_in_file;
    // in ultra rapid, extract only the most recent estimation data 
    for(int j = n/2;j < n/2 + n/8;++j){
      if(j%(num_of_sat + 1) == 0) data.push_back({tmp_data.at(j), vector<string>()});
      else data.back().second.push_back(tmp_data.at(j));
    }
  }
}

void GnssSat_coordinate::ProcessData(vector<vector<string>>& raw_file, vector<string>& data)
{
  for(int i = 0;i < raw_file.size();++i){
    for(int j = 0;j < raw_file.at(i).size();++j){
      if(raw_file.at(i).at(j).substr(0, 3) == "AS ") data.push_back(raw_file.at(i).at(j));
    }
  }
}

void GnssSat_position::Init(vector<vector<string>>& file,
                            int interpolation_method,
                            int interpolation_number,
                            bool ur_flag)
{
  interpolation_number_ = interpolation_number;
  vector<pair<string, vector<string>>> data; //vector of pair, which is string of clock and data of satllites
  if(ur_flag) ProcessData(file, data, ur_flag);
  else ProcessData(file, data);

  //拡張
  gnss_sat_table_ecef_.assign(data.size(), vector<libra::Vector<3>>(all_sat_num_, libra::Vector<3>(0.0)));
  gnss_sat_table_eci_.assign(data.size(), vector<libra::Vector<3>>(all_sat_num_, libra::Vector<3>(0.0)));
  time_table_.resize(data.size());
  available_table_.assign(data.size(), vector<bool>(all_sat_num_, false));
  time_and_index_list_.resize(all_sat_num_);

  for(int i = 0;i < data.size();++i){
    string time_str = data.at(i).first;
    const auto& sp3_str = data.at(i).second;
    istringstream iss_time_str{time_str};
    vector<string> s;
    for(int j = 0;j < 7;++j){
      string tmp;
      iss_time_str >> tmp;
      s.push_back(tmp);
    }
    tm *time_tm = initilized_tm();
    time_tm->tm_year = stoi(s.at(1)) - 1900;
    time_tm->tm_mon = stoi(s.at(2)) - 1; //0 - 11
    time_tm->tm_mday = stoi(s.at(3));
    time_tm->tm_hour = stoi(s.at(4));
    time_tm->tm_min = stoi(s.at(5));
    time_tm->tm_sec = (int)(stod(s.at(6)) + 1e-4);
    double unix_time = (double)mktime(time_tm);
    std::free(time_tm);
    time_table_.at(i) = unix_time;
    if(i > 0) time_interval_ = min(time_interval_, time_table_.at(i) - time_table_.at(i-1));
    double jd;
    jday(stoi(s.at(1)), stoi(s.at(2)), stoi(s.at(3)), stoi(s.at(4)), stoi(s.at(5)), stod(s.at(6)), jd);
    double gs_time_ = gstime(jd);
    double cos_ = cos(gs_time_);
    double sin_ = sin(gs_time_);

    for(int j = 0;j < sp3_str.size();++j){
      istringstream iss{sp3_str.at(j)};
      vector<string> ss;
      for(int k = 0;k < 5;++k){
        string tmp;
        iss >> tmp;
        ss.push_back(tmp);
      }
      int sat_id = GetIndexFromID(ss.front());

      bool available_flag = true;
      for(int k = 0;k < 3;++k){
        if(std::abs(stod(ss.at(k + 1)) - nan99) < 1.0){
          available_flag = false;
          break;
        }
      }
      available_table_.at(i).at(sat_id) = available_flag;

      libra::Vector<3> ecef_position(0.0);
      libra::Vector<3> eci_position(0.0);
      if(available_flag){
        for(int k = 0;k < 3;++k){
          ecef_position[k] = stod(ss.at(k + 1));
        }
        //[km] -> [m]
        ecef_position *= 1000.0;

        double x = ecef_position(0);
        double y = ecef_position(1);
        double z = ecef_position(2);

        eci_position(0) = cos_*x - sin_*y;
        eci_position(1) = sin_*x + cos_*y;
        eci_position(2) = z;

        time_and_index_list_.at(sat_id).emplace_back(unix_time, i);
      }

      gnss_sat_table_ecef_.at(i).at(sat_id) = ecef_position;
      gnss_sat_table_eci_.at(i).at(sat_id) = eci_position;
    }
  }
}

void GnssSat_position::SetUp(const double start_unix_time, const double step_sec)
{
  step_sec_ = step_sec;

  gnss_sat_ecef_.assign(all_sat_num_, Vector<3>(0.0));
  gnss_sat_eci_.assign(all_sat_num_, Vector<3>(0.0));
  validate_.assign(all_sat_num_, false);

  nearest_index_.resize(all_sat_num_);
  time_vector_.resize(all_sat_num_);

  ecef_.resize(all_sat_num_);
  eci_.resize(all_sat_num_);

  for(int i = 0;i < all_sat_num_;++i){
    if(!time_and_index_list_.at(i).size()){
      validate_.at(i) = false;
      continue;
    }

    int list_index = lower_bound(time_and_index_list_.at(i).begin(), time_and_index_list_.at(i).end(), make_pair(start_unix_time, 0)) - time_and_index_list_.at(i).begin();
    double now_time = time_and_index_list_.at(i).at(list_index).first;

    if(interpolation_number_%2 && list_index != 0){
      double pre_time = time_and_index_list_.at(i).at(list_index - 1).first;
      if(std::abs(start_unix_time - pre_time) < std::abs(start_unix_time - now_time)) --list_index;
    }

    nearest_index_.at(i) = list_index;

    //for both even and odd: 2n+1 -> [-n, n] 2n -> [-n, n)
    for(int j = -interpolation_number_/2;j < (interpolation_number_ + 1)/2;++j){
      int now_index = list_index + j;
      if(now_index < 0 || now_index >= time_and_index_list_.at(i).size()) continue;

      const auto& now_pair = time_and_index_list_.at(i).at(now_index);
      const int table_index = now_pair.second;
      time_vector_.at(i).push_back(now_pair.first);
      ecef_.at(i).push_back(gnss_sat_table_ecef_.at(table_index).at(i));
      eci_.at(i).push_back(gnss_sat_table_eci_.at(table_index).at(i));
    }

    if(ecef_.at(i).size() != interpolation_number_) continue;
    double time_period_length = time_vector_.at(i).back() - time_vector_.at(i).front();
    if(time_period_length >= 4.0*60.0*60.0) continue;
    else validate_.at(i) = true;
    
    const auto& nearest_pair = time_and_index_list_.at(i).at(list_index);
    double nearest_unix_time = nearest_pair.first;
    int table_index = nearest_pair.second;
    if(std::abs(start_unix_time - nearest_unix_time) < step_sec/2.0){
      gnss_sat_ecef_.at(i) = gnss_sat_table_ecef_.at(table_index).at(i);
      gnss_sat_eci_.at(i) = gnss_sat_table_eci_.at(table_index).at(i);
      continue;
    }

    if(validate_.at(i)){
      gnss_sat_ecef_.at(i) = TrigonometricInterpolation(time_vector_.at(i), ecef_.at(i), start_unix_time);
      gnss_sat_eci_.at(i) = TrigonometricInterpolation(time_vector_.at(i), eci_.at(i), start_unix_time);
    }
  }
}

void GnssSat_position::Update(const double now_unix_time)
{
  for(int i = 0;i < all_sat_num_;++i){
    if(!time_and_index_list_.at(i).size()){
      validate_.at(i) = false;
      continue;
    }

    int list_index = nearest_index_.at(i);
    double pre_unix = time_and_index_list_.at(i).at(list_index).first;
    double post_unix;
    if(list_index + 1 < time_and_index_list_.at(i).size()) post_unix = time_and_index_list_.at(i).at(list_index + 1).first; 
    else post_unix = 0;

    if(std::abs(now_unix_time - post_unix) < std::abs(now_unix_time - pre_unix)){
      ++list_index;
      nearest_index_.at(i) = list_index;

      time_vector_.at(i).clear();
      ecef_.at(i).clear();
      eci_.at(i).clear();

      //for both even and odd: 2n+1 -> [-n, n] 2n -> [-n, n)
      for(int j = -interpolation_number_/2;j < (interpolation_number_ + 1)/2;++j){
        int now_index = list_index + j;
        if(now_index < 0 || now_index >= time_and_index_list_.at(i).size()) continue;

        const auto& now_pair = time_and_index_list_.at(i).at(now_index);
        const int table_index = now_pair.second;
        time_vector_.at(i).push_back(now_pair.first);
        ecef_.at(i).push_back(gnss_sat_table_ecef_.at(table_index).at(i));
        eci_.at(i).push_back(gnss_sat_table_eci_.at(table_index).at(i));
      }
    }

    if(ecef_.at(i).size() != interpolation_number_){
      validate_.at(i) = false;
      continue;
    }

    double time_period_length = time_vector_.at(i).back() - time_vector_.at(i).front();
    if(time_period_length >= 4.0*60.0*60.0){
      validate_.at(i) = false;
      continue;
    }else validate_.at(i) = true;

    const auto& nearest_pair = time_and_index_list_.at(i).at(list_index);
    double nearest_unix_time = nearest_pair.first;
    int table_index = nearest_pair.second;

    if(validate_.at(i)){
      if(std::abs(now_unix_time - nearest_unix_time) < step_sec_/2.0){
        gnss_sat_ecef_.at(i) = gnss_sat_table_ecef_.at(table_index).at(i);
        gnss_sat_eci_.at(i) = gnss_sat_table_eci_.at(table_index).at(i);
      }else{
        gnss_sat_ecef_.at(i) = TrigonometricInterpolation(time_vector_.at(i), ecef_.at(i), now_unix_time);
        gnss_sat_eci_.at(i) = TrigonometricInterpolation(time_vector_.at(i), eci_.at(i), now_unix_time);
      } 
    }
  }
}

libra::Vector<3> GnssSat_position::GetSatEcef(int sat_id) const
{
  if(sat_id >= all_sat_num_) return Vector<3>(0.0);
  return gnss_sat_ecef_.at(sat_id);
}

libra::Vector<3> GnssSat_position::GetSatEci(int sat_id) const
{
  if(sat_id >= all_sat_num_) return Vector<3>(0.0);
  return gnss_sat_eci_.at(sat_id);
}

void GnssSat_clock::Init(vector<vector<string>>& file, string file_extension,
                         int interpolation_number, bool ur_flag)
{
  interpolation_number_ = interpolation_number;
  if(file_extension == ".sp3"){
    vector<pair<string, vector<string>>> data; //vector of pair, which is string of clock and data of satllites
    if(ur_flag) ProcessData(file, data, ur_flag);
    else ProcessData(file, data);

    gnss_sat_clock_table_.assign(data.size(), vector<double>(all_sat_num_, 0));
    time_table_.resize(data.size());
    available_table_.assign(data.size(), vector<bool>(all_sat_num_, false));
    time_and_index_num_.assign(all_sat_num_, 0);

    for(int i = 0;i < data.size();++i){
      string time_str = data.at(i).first;
      const auto& sp3_str = data.at(i).second;
      istringstream iss_time_str{time_str};
      vector<string> s;
      for(int j = 0;j < 7;++j){
        string tmp;
        iss_time_str >> tmp;
        s.push_back(tmp);
      }
      tm* time_tm = initilized_tm();
      time_tm->tm_year = stoi(s.at(1)) - 1900;
      time_tm->tm_mon = stoi(s.at(2)) - 1; //0 - 11
      time_tm->tm_mday = stoi(s.at(3));
      time_tm->tm_hour = stoi(s.at(4));
      time_tm->tm_min = stoi(s.at(5));
      time_tm->tm_sec = (int)(stod(s.at(6)) + 1e-4);
      double unix_time = (double)mktime(time_tm);
      time_table_.at(i) = unix_time;
      if(i > 0) time_interval_ = min(time_interval_, time_table_.at(i) - time_table_.at(i-1));
      std::free(time_tm);

      for(int j = 0;j < sp3_str.size();++j){
        istringstream iss{sp3_str.at(j)};
        vector<string> ss;
        for(int k = 0;k < 5;++k){
          string tmp;
          iss >> tmp;
          ss.push_back(tmp);
        }
        int sat_id = GetIndexFromID(ss.front());

        bool available_flag = true;
        double clock = stod(ss.at(4));
        if(std::abs(clock - nan99) < 1.0) available_flag = false;
        available_table_.at(i).at(sat_id) = available_flag;

        if(available_flag) ++time_and_index_num_.at(sat_id);
        gnss_sat_clock_table_.at(i).at(sat_id) = clock*(speed_of_light*1e-6);
      }
    }
  }else{
    vector<string> data;
    if(ur_flag){
      cout << "clock settings has something wrong" << endl;
      exit(1);
    }
    else ProcessData(file, data);

    time_and_index_num_.assign(all_sat_num_, 0);
    
    for(int i = 0;i < data.size();++i){
      istringstream iss{data.at(i)};
      vector<string> s;
      for(int j = 0;j < 11;++j){
        string tmp;
        iss >> tmp;
        s.push_back(tmp);
      }
      
      tm *time_tm = initilized_tm();
      time_tm->tm_year = stoi(s.at(2)) - 1900;
      time_tm->tm_mon = stoi(s.at(3)) - 1; //0 - 11
      time_tm->tm_mday = stoi(s.at(4));
      time_tm->tm_hour = stoi(s.at(5));
      time_tm->tm_min = stoi(s.at(6));
      time_tm->tm_sec = (int)(stod(s.at(7)) + 1e-4);
      double unix_time = (double)mktime(time_tm);
      std::free(time_tm);
      if(!time_table_.size() || std::abs(time_table_.back() - unix_time) > 1e-4){
        if(time_table_.size()){
          time_interval_ = min(time_interval_, unix_time - time_table_.back());
        }

        time_table_.push_back(unix_time);
        gnss_sat_clock_table_.push_back(vector<double>(all_sat_num_, 0));
        available_table_.push_back(vector<bool>(all_sat_num_, false));
      }

      int sat_id = GetIndexFromID(s.at(1));
      double clock_bias = stod(s.at(9))*speed_of_light; //[s] -> [m]
      available_table_.back().at(sat_id) = true;
      ++time_and_index_num_.at(sat_id);
      gnss_sat_clock_table_.back().at(sat_id) = clock_bias;
    }
  }
}

void GnssSat_clock::SetUp(const double start_unix_time, const double step_sec)
{
  step_sec_ = step_sec;

  gnss_sat_clock_.resize(all_sat_num_);
  validate_.assign(all_sat_num_, false);

  nearest_index_.resize(all_sat_num_);
  time_vector_.resize(all_sat_num_);

  clock_bias_.resize(all_sat_num_);

  for(int i = 0;i < all_sat_num_;++i){
    if(!time_and_index_num_.at(i)){
      validate_.at(i) = false;
      continue;
    }

    int table_index = lower_bound(time_table_.begin(), time_table_.end(), start_unix_time) - time_table_.begin();
    double now_time = time_table_.at(table_index);

    if(interpolation_number_%2 && table_index != 0){
      double pre_time = time_table_.at(table_index - 1);
      if(std::abs(start_unix_time - pre_time) < std::abs(start_unix_time - now_time)) --table_index;
    }
    
    nearest_index_.at(i) = table_index;

    //for both even and odd: 2n+1 -> [-n, n] 2n -> [-n, n)
    for(int j = -interpolation_number_/2;j < (interpolation_number_ + 1)/2;++j){
      int index = table_index + j;
      if(index < 0 || index >= time_table_.size()) continue;
      if(!available_table_.at(index).at(i)) continue;

      time_vector_.at(i).push_back(time_table_.at(index));
      clock_bias_.at(i).push_back(gnss_sat_clock_table_.at(index).at(i));
    }

    if(clock_bias_.at(i).size() != interpolation_number_) continue;
    else validate_.at(i) = true;

    if(validate_.at(i)){
      if(std::abs(start_unix_time - time_table_.at(table_index)) < step_sec/2.0){
        gnss_sat_clock_.at(i) = gnss_sat_clock_table_.at(table_index).at(i);
      }else{
        gnss_sat_clock_.at(i) = LagrangeInterpolation(time_vector_.at(i), clock_bias_.at(i), start_unix_time);
      }    
    }
  }
}

void GnssSat_clock::Update(const double now_unix_time)
{
  for(int i = 0;i < all_sat_num_;++i){
    if(!time_and_index_num_.at(i)){
      validate_.at(i) = false;
      continue;
    }

    int table_index = nearest_index_.at(i);
    double pre_unix = time_table_.at(table_index);
    double post_unix;
    if(table_index + 1 < time_table_.size()) post_unix = time_table_.at(table_index + 1);
    else post_unix = 0;

    if(std::abs(now_unix_time - post_unix) < std::abs(now_unix_time - pre_unix)){
      ++table_index;
      nearest_index_.at(i) = table_index;

      time_vector_.at(i).clear();
      clock_bias_.at(i).clear();

      //for both even and odd: 2n+1 -> [-n, n] 2n -> [-n, n) //in clock_bias, more strict.
      for(int j = -interpolation_number_/2;j < (interpolation_number_ + 1)/2;++j){
        int index = table_index + j;
        if(index < 0 || index >= time_table_.size()) continue;
        if(!available_table_.at(index).at(i)) continue;
        time_vector_.at(i).push_back(time_table_.at(index));
        clock_bias_.at(i).push_back(gnss_sat_clock_table_.at(index).at(i));
      }
    }

    if(clock_bias_.at(i).size() != interpolation_number_){
      validate_.at(i) = false;
      continue;
    }else validate_.at(i) = true;

    if(validate_.at(i)){
      if(std::abs(now_unix_time - time_table_.at(table_index)) < step_sec_/2.0){
        gnss_sat_clock_.at(i) = gnss_sat_clock_table_.at(table_index).at(i);
      }else{
        gnss_sat_clock_.at(i) = LagrangeInterpolation(time_vector_.at(i), clock_bias_.at(i), now_unix_time);
      }
    }  
  }
}

double GnssSat_clock::GetSatClock(int sat_id) const
{
  if(sat_id >= all_sat_num_) return 0.0;
  return gnss_sat_clock_.at(sat_id);
}

GnssSat_Info::GnssSat_Info() {}
void GnssSat_Info::Init(vector<vector<string>>& position_file,
                               int position_interpolation_method,
                               int position_interpolation_number,
                               bool position_ur_flag,

                               vector<vector<string>>& clock_file,
                               string clock_file_extension,
                               int clock_interpolation_number,
                               bool clock_ur_flag)
{
  position_.Init(position_file, position_interpolation_method, 
               position_interpolation_number, position_ur_flag);
  clock_.Init(clock_file, clock_file_extension,
            clock_interpolation_number, clock_ur_flag);
}

void GnssSat_Info::SetUp(const double start_unix_time, const double step_sec)
{
  position_.SetUp(start_unix_time, step_sec);
  clock_.SetUp(start_unix_time, step_sec);
}

void GnssSat_Info::Update(const double now_unix_time)
{
  position_.Update(now_unix_time);
  clock_.Update(now_unix_time);
}

int GnssSat_Info::GetNumOfSatellites() const
{
  if(position_.GetNumOfSatellites() == clock_.GetNumOfSatellites()) return position_.GetNumOfSatellites();
  else{
    cout << "Num Of Gnss Satellites has something wrong" << endl;
    exit(1);
    return 0;
  }
}

bool GnssSat_Info::GetWhetherValid(int sat_id) const
{
  if(position_.GetWhetherValid(sat_id) && clock_.GetWhetherValid(sat_id)) return true;
  return false;
}

libra::Vector<3> GnssSat_Info::GetSatellitePositionEcef(int sat_id) const
{
  return position_.GetSatEcef(sat_id);
}

libra::Vector<3> GnssSat_Info::GetSatellitePositionEci(int sat_id) const
{
  return position_.GetSatEci(sat_id);
}

double GnssSat_Info::GetSatelliteClock(int sat_id) const
{
  return clock_.GetSatClock(sat_id);
}

GnssSatellites::GnssSatellites(bool is_calc_enabled)//: ofs_true("true.csv"), ofs_esti("esti.csv"), ofs_sa("sa.csv")
{
  is_calc_enabled_ = is_calc_enabled;
}

bool GnssSatellites::IsCalcEnabled() const
{
  return is_calc_enabled_;
}

void GnssSatellites::Init(vector<vector<string>>& true_position_file,
                          int true_position_interpolation_method,
                          int true_position_interpolation_number,
                          bool true_position_ur_flag,

                          vector<vector<string>>& true_clock_file,
                          string true_clock_file_extension,
                          int true_clock_interpolation_number,
                          bool true_clock_ur_flag,

                          vector<vector<string>>& estimate_position_file,
                          int estimate_position_interpolation_method,
                          int estimate_position_interpolation_number,
                          bool estimate_position_ur_flag,

                          vector<vector<string>>& estimate_clock_file,
                          string estimate_clock_file_extension,
                          int estimate_clock_interpolation_number,
                          bool estimate_clock_ur_flag)
{
  true_info_.Init(true_position_file,
                  true_position_interpolation_method,
                  true_position_interpolation_number,
                  true_position_ur_flag,

                  true_clock_file,
                  true_clock_file_extension,
                  true_clock_interpolation_number,
                  true_clock_ur_flag);
  
  estimate_info_.Init(estimate_position_file,
                      estimate_position_interpolation_method,
                      estimate_position_interpolation_number,
                      estimate_position_ur_flag,

                      estimate_clock_file,
                      estimate_clock_file_extension,
                      estimate_clock_interpolation_number,
                      estimate_clock_ur_flag);
  
  return;
}

void GnssSatellites::SetUp(const SimTime* sim_time)
{
  if(!IsCalcEnabled()) return;

  tm *start_tm = initilized_tm();
  start_tm->tm_year = sim_time->GetStartYear() - 1900;
  start_tm->tm_mon = sim_time->GetStartMon() - 1;
  start_tm->tm_mday = sim_time->GetStartDay();
  start_tm->tm_hour = sim_time->GetStartHr();
  start_tm->tm_min = sim_time->GetStartMin();
  double start_sec = sim_time->GetStartSec();
  start_tm->tm_sec = (int)start_sec;
  double unix_time = (double)mktime(start_tm) + start_sec - floor(start_sec);
  std::free(start_tm);
  true_info_.SetUp(unix_time, sim_time->GetStepSec());
  estimate_info_.SetUp(unix_time, sim_time->GetStepSec());

  start_unix_time_ = unix_time;

  return;
}

void GnssSatellites::Update(const SimTime* sim_time)
{
  if (!IsCalcEnabled()) return;
  
  double elapsed_sec = sim_time->GetElapsedSec();

  true_info_.Update(elapsed_sec + start_unix_time_);
  estimate_info_.Update(elapsed_sec + start_unix_time_);

  //DebugOutput();
  return;
}

int GnssSatellites::GetNumOfSatellites() const
{
  return estimate_info_.GetNumOfSatellites();
}

bool GnssSatellites::GetWhetherValid(int sat_id) const
{
  if(sat_id >= GetNumOfSatellites()) return false;

  if(true_info_.GetWhetherValid(sat_id) && estimate_info_.GetWhetherValid(sat_id)) return true;
  else return false;
}

double GnssSatellites::GetStartUnixTime() const
{
  return start_unix_time_;
}

const GnssSat_Info& GnssSatellites::Get_true_info() const
{
  return true_info_;
}

const GnssSat_Info& GnssSatellites::Get_estimate_info() const
{
  return estimate_info_;
}

libra::Vector<3> GnssSatellites::GetSatellitePositionEcef(const int sat_id) const
{
  //sat_id is wrong or not valid
  if(sat_id >= GetNumOfSatellites() || !GetWhetherValid(sat_id)){
    libra::Vector<3> res(0);
    return res;
  }

  return estimate_info_.GetSatellitePositionEcef(sat_id);
}

libra::Vector<3> GnssSatellites::GetSatellitePositionEci(const int sat_id) const
{
  //sat_id is wrong or not valid
  if(sat_id >= GetNumOfSatellites() || !GetWhetherValid(sat_id)){
    libra::Vector<3> res(0);
    return res;
  }

  return estimate_info_.GetSatellitePositionEci(sat_id);
}

double GnssSatellites::GetSatelliteClock(const int sat_id) const
{
  if(sat_id >= GetNumOfSatellites() || !GetWhetherValid(sat_id)){
    return 0.0;
  }
  
  return estimate_info_.GetSatelliteClock(sat_id) + clock_bias_bias_;
}

double GnssSatellites::GetPseudoRangeECEF(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const
{
  //sat_id is wrong or not validate
  if(sat_id >= GetNumOfSatellites() || !GetWhetherValid(sat_id)) return 0.0;

  double res = 0.0;
  auto gnss_position = true_info_.GetSatellitePositionEcef(sat_id);
  for(int i = 0;i < 3;++i){
    res += pow(rec_position(i) - gnss_position(i), 2.0);
  }
  res = sqrt(res);

  // clock bias
  res += rec_clock - true_info_.GetSatelliteClock(sat_id);

  //ionospheric delay
  const double ionospheric_delay = AddIonosphericDelay(sat_id, rec_position, frequency, ECEF);
  
  res += ionospheric_delay;

  return res;
}

double GnssSatellites::GetPseudoRangeECI(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const
{
  //sat_id is wrong or not validate
  if(sat_id >= GetNumOfSatellites() || !GetWhetherValid(sat_id)) return 0.0;

  double res = 0.0;
  auto gnss_position = true_info_.GetSatellitePositionEci(sat_id);
  for(int i = 0;i < 3;++i){
    res += pow(rec_position(i) - gnss_position(i), 2.0);
  }
  res = sqrt(res);

  //clock bias
  res += rec_clock - true_info_.GetSatelliteClock(sat_id);

  //ionospheric delay
  const double ionospheric_delay = AddIonosphericDelay(sat_id, rec_position, frequency, ECI);
  
  res += ionospheric_delay;

  return res;
}

pair<double, double> GnssSatellites::GetCarrierPhaseECEF(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const
{
  //sat_id is wrong or not validate
  if(sat_id >= GetNumOfSatellites() || !GetWhetherValid(sat_id)) return {0.0, 0.0};

  double res = 0.0;
  auto gnss_position = true_info_.GetSatellitePositionEcef(sat_id);
  for(int i = 0;i < 3;++i){
    res += pow(rec_position(i) - gnss_position(i), 2.0);
  }
  res = sqrt(res);

  // clock bias
  res += rec_clock - true_info_.GetSatelliteClock(sat_id);

  //ionospheric delay
  const double ionospheric_delay = AddIonosphericDelay(sat_id, rec_position, frequency, ECEF);
  
  res -= ionospheric_delay;
  
  // wavelength
  double lambda = speed_of_light*1e-6/frequency;
  double cycle = res/lambda;

  double bias = floor(cycle);
  cycle -= bias;

  return {cycle, bias};
}

pair<double, double> GnssSatellites::GetCarrierPhaseECI(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const
{
  //sat_id is wrong or not validate
  if(sat_id >= GetNumOfSatellites() || !GetWhetherValid(sat_id)) return {0.0, 0.0};

  double res = 0.0;
  auto gnss_position = true_info_.GetSatellitePositionEci(sat_id);
  for(int i = 0;i < 3;++i){
    res += pow(rec_position(i) - gnss_position(i), 2.0);
  }
  res = sqrt(res);

  //clock bias
  res += rec_clock - true_info_.GetSatelliteClock(sat_id);

  //ionospheric delay
  const double ionospheric_delay = AddIonosphericDelay(sat_id, rec_position, frequency, ECI);
  
  res -= ionospheric_delay;

  // wavelength
  double lambda = speed_of_light*1e-6/frequency;
  double cycle = res/lambda;

  double bias = floor(cycle);
  cycle -= bias;

  return {cycle, bias};
}

// for Ionospheric delay I[m]
double GnssSatellites::AddIonosphericDelay(const int sat_id, const libra::Vector<3> rec_position, const double frequency, const bool flag) const
{
  //sat_id is wrong or not validate
  if(sat_id >= GetNumOfSatellites() || !GetWhetherValid(sat_id)) return 0.0;

  const double Earth_hemisphere = 6378.1; //[km]

  double altitude = 0.0;
  for(int i = 0;i < 3;++i) altitude += pow(rec_position[i], 2.0);
  altitude = sqrt(altitude);
  altitude = altitude/1000.0 - Earth_hemisphere; //[m -> km]
  if(altitude >= 1000.0) return 0.0; //there is no Ionosphere above 1000km

  libra::Vector<3> gnss_position;
  if(flag == ECEF) gnss_position = true_info_.GetSatellitePositionEcef(sat_id);
  else if(flag == ECI) gnss_position = true_info_.GetSatellitePositionEci(sat_id); 

  double angle_rad = angle(rec_position, gnss_position - rec_position);
  const double default_delay = 20.0; //[m] default delay
  double delay = default_delay*(1000.0 - altitude)/1000.0/cos(angle_rad); //set the maximum height as 1000.0. Divide by cos because the slope makes it longer.
  const double default_frequency = 1500.0; //[MHz]
  // Ionospheric delay is inversely proportional to the square of the frequency
  delay *= pow(default_frequency/frequency, 2.0);

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

void GnssSatellites::DebugOutput()
{
  for(int sat_id = 0;sat_id < 32;++sat_id){
    if(true_info_.GetWhetherValid(sat_id)){
      auto true_pos = true_info_.GetSatellitePositionEci(sat_id);
      for(int i = 0;i < 3;++i){
        //ofs_true << fixed << setprecision(10) << true_pos[i] << ",";
      }
      auto true_clock = true_info_.GetSatelliteClock(sat_id);
      //ofs_true << true_clock << ",";
    }else{
      for(int i = 0;i < 4;++i){
        //ofs_true << 0.0 << ",";
      }
    }

    if(estimate_info_.GetWhetherValid(sat_id)){
      auto esti_pos = estimate_info_.GetSatellitePositionEci(sat_id);
      for(int i = 0;i < 3;++i){
        //ofs_esti << fixed << setprecision(10) << esti_pos[i] << ",";
      }
      auto esti_clock = estimate_info_.GetSatelliteClock(sat_id);
      //ofs_esti << esti_clock << ",";
    }else{
      for(int i = 0;i < 4;++i){
        //ofs_esti << 0.0 << ",";
      }
    }

    if(GetWhetherValid(sat_id)){
      auto true_pos = true_info_.GetSatellitePositionEci(sat_id);
      auto true_clock = true_info_.GetSatelliteClock(sat_id);
      auto esti_pos = estimate_info_.GetSatellitePositionEci(sat_id);
      auto esti_clock = estimate_info_.GetSatelliteClock(sat_id);

      for(int i = 0;i < 3;++i){
        //ofs_sa << fixed << setprecision(10) << esti_pos[i] - true_pos[i] << ",";
      }
      //ofs_sa << fixed << setprecision(10) << esti_clock - true_clock << ",";
    }else{
      for(int i = 0;i < 4;++i){
        //ofs_sa << 0.0 << ",";
      }
    }
  }

  //ofs_true << endl;
  //ofs_esti << endl;
  //ofs_sa << endl;

  return;
}
