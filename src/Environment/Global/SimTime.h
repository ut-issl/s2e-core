#ifndef __SIMULATION_TIME_H__
#define __SIMULATION_TIME_H__

#ifdef WIN32
	#define _WINSOCKAPI_    // stops windows.h including winsock.h
#endif

#include <string>
// #include <time.h>
#include <chrono>

#include "../../Interface/LogOutput/ILoggable.h"
#include "../../Library/sgp4/sgp4unit.h"
#include "../../Library/sgp4/sgp4io.h"
#include "../../Library/sgp4/sgp4ext.h"

using namespace std;

struct TimeState
{
  bool running = false;
  bool finish = false;
  bool log_output = false;
  bool disp_output = true;
};

// UTC (calendar day) struct
struct UTC
{
  unsigned int year = 2000;
  unsigned int month = 1;
  unsigned int day = 1;
  unsigned int hour = 0;
  unsigned int min = 0;
  double sec = 0.0;
};

// シミュレーション上の時間を管理するシングルトン
class SimTime: public ILoggable
{
public:
  SimTime(
    const double end_sec,
    const double step_sec,
    const double attitude_update_interval_sec,
    const double attitude_rk_step_sec,
    const double orbit_update_interval_sec,
    const double orbit_rk_step_sec,
    const double thermal_update_interval_sec,
    const double thermal_rk_step_sec,
    const double compo_propagate_step_sec,
    const double log_output_interval_sec,
    const char* start_ymdhms,
    const double sim_speed);
  ~SimTime();

  void SetParameters(void); // Simulation開始前処理（Monte-Carlo Simulationの際は毎回呼ばれる予定）
  void UpdateTime(void);		// 時刻の更新
  void ResetClock(void);

  // Get functions
  inline const TimeState GetState(void) const { return state_; };
  inline const double GetElapsedSec(void) const { return elapsed_time_sec_; };
  inline const double GetStepSec(void) const { return step_sec_; };
  inline const double GetAttitudeUpdateIntervalSec(void) const { return attitude_update_interval_sec_; };
  inline const bool GetAttitudePropagateFlag(void) const { return attitude_update_flag_; };
  inline const double GetAttitudeRKStepSec() const { return attitude_rk_step_sec_; }
  inline const double GetOrbitUpdateIntervalSec(void) const { return orbit_update_interval_sec_; };
  inline const bool GetOrbitPropagateFlag(void) const { return orbit_update_flag_; };
  inline const double GetOrbitRKStepSec() const { return orbit_rk_step_sec_; }
  inline const double GetThermalUpdateIntervalSec(void) const { return thermal_update_interval_sec_; };
  inline const bool GetThermalPropagateFlag(void) const { return thermal_update_flag_; };
  inline const double GetThermalRKStepSec() const { return thermal_rk_step_sec_; }
  inline const double GetCompoStepSec(void) const { return compo_update_interval_sec_; };
  inline const bool GetCompoUpdateFlag() const { return compo_update_flag_; }
  inline const int GetCompoPropagateFrequency(void) const { return compo_propagate_frequency_; };

  inline const double GetEndSec(void) const { return end_sec_; };
  inline const int GetProgressionRate(void) const { return (int)floor((elapsed_time_sec_ / end_sec_ * 100)); };
  inline const double GetCurrentJd(void) const { return current_jd_; };
  inline const double GetCurrentSidereal(void) const { return current_sidereal_; };
  inline const double GetCurrentDecyear(void) const { return current_decyear_; };
  inline const UTC GetCurrentUTC(void) const { return current_utc_; };
  inline const int GetStartYear(void) const { return start_year_; };
  inline const int GetStartMon(void) const { return start_mon_; };
  inline const int GetStartDay(void) const { return start_day_; };
  inline const int GetStartHr(void) const { return start_hr_; };
  inline const int GetStartMin(void) const { return start_min_; };
  inline const double GetStartSec(void) const { return start_sec_; };
  // logs
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;
  //debug
  void PrintStartDateTime(void) const;

private:
  //変動値
  double elapsed_time_sec_;
  double current_jd_;      //ユリウス日
  double current_sidereal_; //グリニッジ平均恒星時
  double current_decyear_; //Decimal Year(yearの小数点表記)
  UTC	 current_utc_; // UTC calendar day

  int attitude_update_counter_;
  bool attitude_update_flag_;
  int orbit_update_counter_;
  bool orbit_update_flag_;
  int thermal_update_counter_;
  bool thermal_update_flag_;
  int compo_update_counter_;
  bool compo_update_flag_;
  int log_counter_;
  int disp_counter_;
  TimeState state_;
  chrono::system_clock::time_point clock_start_time_millisec_;
  // chrono::system_clock::time_point clock_elapsed_time_millisec_; //実時間でのシミュレーション実行時間

                                        //固定値
  double end_sec_;	//simulation開始から終了までの時刻
  double step_sec_;	//simulation刻み時刻
  double attitude_update_interval_sec_;
  double attitude_rk_step_sec_;
  double orbit_update_interval_sec_;
  double orbit_rk_step_sec_;
  double thermal_update_interval_sec_;
  double thermal_rk_step_sec_;
  double compo_update_interval_sec_;
  int compo_propagate_frequency_;
  double log_output_interval_sec_;
  double disp_period_;	//コンソール出力頻度
  double start_jd_;
  int start_year_;
  int start_mon_;
  int start_day_;
  int start_hr_;
  int start_min_;
  double start_sec_;
  double sim_speed_; //実時間に対するシミュレーションの進行速度（負の場合は実時間を考慮しない）

  void InitializeState();
  void AssertTimeStepParams();
  void ConvJDtoCalndarDay(const double JD); // wrapper function of invjday @ sgp4ext for interface adjustment
};
#endif //__SIMULATION_TIME_H__
