#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <sstream>
#include <math.h>
#include <cassert>

#include "SimTime.h"
#ifdef WIN32
  #include <Windows.h>
#else
  #include <errno.h>
#endif

using namespace std;

SimTime::SimTime(
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
  const double sim_speed)
{
  end_sec_ = end_sec;
  step_sec_ = step_sec;
  attitude_update_interval_sec_ = attitude_update_interval_sec;
  attitude_rk_step_sec_ = attitude_rk_step_sec;
  orbit_update_interval_sec_ = orbit_update_interval_sec;
  orbit_rk_step_sec_ = orbit_rk_step_sec;
  thermal_update_interval_sec_ = thermal_update_interval_sec;
  thermal_rk_step_sec_ = thermal_rk_step_sec;
  log_output_interval_sec_ = log_output_interval_sec;
  compo_update_interval_sec_ = compo_propagate_step_sec;
  compo_propagate_frequency_ = int(1.0 / compo_update_interval_sec_);
  sim_speed_ = sim_speed;
  disp_period_ = (1.0 * end_sec / step_sec / 100);  // 1%毎に更新

//  sscanf_s(start_ymdhms, "%d/%d/%d %d:%d:%lf", &start_year_, &start_mon_, &start_day_, &start_hr_, &start_min_, &start_sec_);
  sscanf(start_ymdhms, "%d/%d/%d %d:%d:%lf", &start_year_, &start_mon_, &start_day_, &start_hr_, &start_min_, &start_sec_);
  jday(start_year_, start_mon_, start_day_, start_hr_, start_min_, start_sec_, start_jd_);
  current_jd_ = start_jd_;
  current_sidereal_ = gstime(current_jd_);
  JdToDecyear(current_jd_, &current_decyear_);
  ConvJDtoCalndarDay(current_jd_);
  AssertTimeStepParams();
  InitializeState();
  SetParameters();
}

SimTime::~SimTime()
{
}

void SimTime::AssertTimeStepParams()
{
  //Runge-Kutta time step must be smaller than its update interval
  assert(attitude_rk_step_sec_ <= attitude_update_interval_sec_);
  assert(orbit_rk_step_sec_ <= orbit_update_interval_sec_);
  assert(thermal_rk_step_sec_ <= thermal_update_interval_sec_);

  //Step time for the entire simulation must be smaller than all of the subroutine step times
  assert(step_sec_ <= attitude_update_interval_sec_);
  assert(step_sec_ <= orbit_update_interval_sec_);
  assert(step_sec_ <= thermal_update_interval_sec_);
  assert(step_sec_ <= compo_update_interval_sec_);
  assert(step_sec_ <= log_output_interval_sec_);
}

void SimTime::SetParameters(void)
{
  elapsed_time_sec_ = 0.0;
  attitude_update_counter_ = 1;
  attitude_update_flag_ = false;
  orbit_update_counter_ = 1;
  orbit_update_flag_ = false;
  thermal_update_counter_ = 1;
  thermal_update_flag_ = false;
  compo_update_counter_ = 1;
  compo_update_flag_ = false;
  log_counter_ = 0;
  disp_counter_ = 0;
  state_.log_output = true;
}

void SimTime::UpdateTime(void)
{
  InitializeState();
  elapsed_time_sec_ += step_sec_;
  if (sim_speed_ > 0)
  {
    chrono::system_clock clk;
    int toWaitTime = (int)(elapsed_time_sec_ * 1000 - 
        chrono::duration_cast<chrono::milliseconds>(clk.now() - clock_start_time_millisec_).count()
        * sim_speed_);
    if (toWaitTime <= 0)
    {
      // PCの処理速度が足りていない場合、この分岐に入る
      cout << "Error: the specified step_sec is too small for this computer.\r\n";

      // カウントアップしてきた経過時刻を強制的に実際の経過時刻に合わせる
      // 意図としては、ブレークポイントからの復帰後に一瞬でリアルタイムに追いつかせるため
      elapsed_time_sec_ = chrono::duration_cast<chrono::seconds>(clk.now() - clock_start_time_millisec_).count() * sim_speed_;
    }
    else
    {
#ifdef WIN32
      Sleep(toWaitTime);
#else
      const struct timespec req = {toWaitTime/1000, (toWaitTime%1000)*1000000};
      nanosleep(&req, NULL);
#endif
    }
  }

  attitude_update_counter_++;
  orbit_update_counter_++;
  thermal_update_counter_++;
  compo_update_counter_++;
  log_counter_++;
  disp_counter_++;

  if (elapsed_time_sec_ > end_sec_)
  {
    state_.finish = true;
  }

  current_jd_ = start_jd_ + elapsed_time_sec_ / (60.0 * 60.0 * 24.0);
  current_sidereal_ = gstime(current_jd_);
  JdToDecyear(current_jd_, &current_decyear_);
  ConvJDtoCalndarDay(current_jd_);

  attitude_update_flag_ = false;
  if (double(attitude_update_counter_) * step_sec_ >= attitude_update_interval_sec_)
  {
    attitude_update_counter_ = 0;
    attitude_update_flag_ = true;
  }

  orbit_update_flag_ = false;
  if (double(orbit_update_counter_) * step_sec_ >= orbit_update_interval_sec_)
  {
    orbit_update_counter_ = 0;
    orbit_update_flag_ = true;
  }

  thermal_update_flag_ = false;
  if (double(thermal_update_counter_) * step_sec_ >= thermal_update_interval_sec_)
  {
    thermal_update_counter_ = 0;
    thermal_update_flag_ = true;
  }

  compo_update_flag_ = false;
  if (double(compo_update_counter_) * step_sec_ >= compo_update_interval_sec_)
  {
    compo_update_counter_ = 0;
    compo_update_flag_ = true;
  }

  if (double(log_counter_) * step_sec_ >= log_output_interval_sec_)
  {
    log_counter_ = 0;
    state_.log_output = true;
  }

  if (disp_counter_ >= disp_period_)
  {
    disp_counter_ -= disp_period_;
    state_.disp_output = true;
  }

  state_.running = true;
}

void SimTime::ResetClock(void)
{
  clock_start_time_millisec_ = chrono::system_clock::now();
}

void SimTime::PrintStartDateTime(void) const
{
  int sec_int = int(start_sec_ + 0.5);
  stringstream s, m, h;
  if (sec_int < 10){ s << 0 << sec_int; }
  else{ s << sec_int; }
  if (start_min_ < 10){ m << 0 << start_min_; }
  else{ m << start_min_; }
  if (start_hr_ < 10){ h << 0 << start_hr_; }
  else{ h << start_hr_; }

  cout << " " << start_year_ << "/" << start_mon_ << "/" << start_day_ << " " << h.str() << ":" << m.str() << ":" << s.str() << "\n";
}

string SimTime::GetLogHeader() const
{
  string str_tmp = "";

  str_tmp += WriteScalar("time", "sec");

  return str_tmp;
}

string SimTime::GetLogValue() const
{
  string str_tmp = "";

  str_tmp += WriteScalar(elapsed_time_sec_);

  return str_tmp;
}

void SimTime::InitializeState()
{
  state_.disp_output = false;
  state_.finish = false;
  state_.log_output = false;
  state_.running = false;
}

// wrapper function of invjday @ sgp4ext for interface adjustment
void SimTime::ConvJDtoCalndarDay(const double JD)
{
  int year, mon, day, hr, min;
  double sec;
  invjday(JD, year, mon, day, hr, min, sec);
  current_utc_.year  = (unsigned int)(year);
  current_utc_.month = (unsigned int)(mon);
  current_utc_.day   = (unsigned int)(day);
  current_utc_.hour  = (unsigned int)(hr);
  current_utc_.min   = (unsigned int)(min);
  current_utc_.sec   = sec;
}
