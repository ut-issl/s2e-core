#ifndef __gnss_satellites_h__
#define __gnss_satellites_h__

#include <vector>
#include <cmath>

#include "../../Library/math/Vector.hpp"
#include "../../Interface/LogOutput/ILoggable.h"

const double speed_of_light = 299792458.0; //[m/s] in vacuum

#define ECEF 0
#define ECI 1

class GnssSatellites_Info
{
  public:
    GnssSatellites_Info() {}
    void Update(vector<libra::Vector<3>>& eci_){
      gnss_satellites_position_eci_ = &eci_;
    }
    const libra::Vector<3>& GetSatellitePositionEci(int sat_id) const{
      return (*gnss_satellites_position_eci_).at(sat_id);
    }
  private:
    vector<libra::Vector<3>>* gnss_satellites_position_eci_;
};

class GnssSatellites : public ILoggable
{
public:
  //CONSTRUCTOR AND DECONSTRUCTOR
  GnssSatellites(int satellite_num, int time_stamp_num, int num_of_days);
  ~GnssSatellites();
  void ReadSP3(vector<vector<string>>& sp3_file);
  void InitTable(int start_hr, int start_min, double start_sec, double elapsed_time_sec); //補間用のTable作成
  void Update(int start_hr, int start_min, double start_sec, double elapsed_time_sec);
  void UpdatePosition(double now_sec);
  void UpdateClock(double now_sec);

  //GET FUNCTIONS
  int GetNumOfSatellites() const;
  bool GetWhetherValid(int sat_id) const;
  const GnssSatellites_Info& Get_true_info() const;

  libra::Vector<3> GetSatellitePositionEcef(const int sat_id) const;
  libra::Vector<3> GetSatellitePositionEci(const int sat_id) const;

  double GetSatelliteClock(const int sat_id) const;

  //Get observation Range:[m] CarrierPhase:[no unit]
  double GetPseudoRangeECEF(const int sat_id, libra::Vector<3> sat_position, double sat_clock, const double frequency) const;
  double GetPseudoRangeECI(const int sat_id, libra::Vector<3> sat_position, double sat_clock, const double frequency) const;
  pair<double, double> GetCarrierPhaseECEF(const int sat_id, libra::Vector<3> sat_position, double sat_clock, const double frequency) const;
  pair<double, double> GetCarrierPhaseECI(const int sat_id, libra::Vector<3> sat_position, double sat_clock, const double frequency) const;

  // FOR LOG OUTPUT // 継承のために
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

  // FOR DEBUG OUTPUT
  void DebugOutput(void);
  
  bool IsCalcEnabled = true;

private:
  int num_of_satellites_;
  int num_of_time_stamps_;
  int num_of_days_;

  const int num_of_points = 9; //should be odds number
  int half_points;

  //gnss_satellites_position [m]
  std::vector<libra::Vector<3>> gnss_satellites_position_ecef_;
  std::vector<libra::Vector<3>> gnss_satellites_position_eci_;

  //clock bais [micro second]
  std::vector<double> gnss_satellites_clock;

  std::vector<std::vector<libra::Vector<3>>> gnss_satellites_position_table_ecef_;
  std::vector<std::vector<libra::Vector<3>>> gnss_satellites_position_table_eci_;
  std::vector<double> side_table_; //ECEF to ECI のため
  std::vector<double> time_table_;

  int pre_index;
  vector<double> time_period;
  double center_time;
  std::vector<vector<double>> eci_x;
  std::vector<vector<double>> eci_y;
  std::vector<vector<double>> eci_z;

  std::vector<vector<double>> ecef_x;
  std::vector<vector<double>> ecef_y;
  std::vector<vector<double>> ecef_z;

  double TrigonometricInterpolation(vector<double> time_period, vector<double> position, double time);
  // for Ionospheric delay かなり雑に電離層遅延を出しています。要改善
  double AddIonosphericDelay(const int sat_id, const libra::Vector<3> sat_position, const double frequency, const bool flag) const;

  GnssSatellites_Info gnss_true;
};
#endif
