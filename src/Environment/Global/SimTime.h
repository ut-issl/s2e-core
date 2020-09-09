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

// シミュレーション上の時間を管理するシングルトン
class SimTime: public ILoggable
{
public:
  SimTime(const double end_sec,
    const double step_sec,
    const double orbit_propagate_step_sec,
    const int log_period,
    const char* start_ymdhms,
    const double sim_speed);
  ~SimTime();

  void SetParameters(void); // Simulation開始前処理（Monte-Carlo Simulationの際は毎回呼ばれる予定）
  void UpdateTime(void);		// 時刻の更新
  void ResetClock(void);

  // Get functions
  TimeState GetState(void) const;
  double GetElapsedSec(void) const;
  double GetStepSec(void) const;
  double GetOrbitStepSec(void) const;
  double GetEndSec(void) const;
  int GetProgressionRate(void) const;
  double GetCurrentJd(void) const;
  bool GetOrbitPropagateFlag(void) const;
  double GetCurrentSidereal(void) const;
  double GetCurrentDecyear(void) const;
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

  int orbit_propagate_counter_;
  bool orbit_propagate_flag_;
  int log_counter_;
  int disp_counter_;
  TimeState state_;
  chrono::system_clock::time_point clock_start_time_millisec_;
  // chrono::system_clock::time_point clock_elapsed_time_millisec_; //実時間でのシミュレーション実行時間

                                        //固定値
  double end_sec_;	//simulation開始から終了までの時刻
  double step_sec_;	//simulation刻み時刻
  double orbit_propagate_step_sec_; //軌道計算用刻み時間
  int log_period_;	//Logファイル出力頻度(ステップ時間=log_period*sim_step_sec)
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
};
#endif //__SIMULATION_TIME_H__
