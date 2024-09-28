/**
 *@file simulation_time.cpp
 *@brief Class to manage simulation time related information
 */

#define _CRT_SECURE_NO_WARNINGS
#include "simulation_time.hpp"

#include <SpiceUsr.h>

#include <cassert>
#include <iostream>
#include <sstream>

#include "setting_file_reader/initialize_file_access.hpp"
#ifdef WIN32
#include <Windows.h>
#else
#include <errno.h>
#endif

using namespace std;

namespace s2e::environment {

SimulationTime::SimulationTime(const double end_sec, const double step_sec, const double attitude_update_interval_sec,
                               const double attitude_rk_step_sec, const double orbit_update_interval_sec, const double orbit_rk_step_sec,
                               const double thermal_update_interval_sec, const double thermal_rk_step_sec, const double compo_propagate_step_sec,
                               const double log_output_interval_sec, const char* start_ymdhms, const double sim_speed) {
  end_sec_ = end_sec;
  step_sec_ = step_sec;
  attitude_update_interval_sec_ = attitude_update_interval_sec;
  attitude_rk_step_sec_ = attitude_rk_step_sec;
  orbit_update_interval_sec_ = orbit_update_interval_sec;
  orbit_rk_step_sec_ = orbit_rk_step_sec;
  thermal_update_interval_sec_ = thermal_update_interval_sec;
  thermal_rk_step_sec_ = thermal_rk_step_sec;
  log_output_interval_sec_ = log_output_interval_sec;
  component_update_interval_sec_ = compo_propagate_step_sec;
  component_propagate_frequency_Hz_ = int(1.0 / component_update_interval_sec_);
  simulation_speed_ = sim_speed;
  display_period_ = (1.0 * end_sec / step_sec / 100);  // Update every 1%
  time_exceeds_continuously_limit_sec_ = 1.0;

  //  sscanf_s(start_ymdhms, "%d/%d/%d %d:%d:%lf", &start_year_, &start_month_, &start_day_, &start_hour_, &start_minute_, &start_sec_);
  sscanf(start_ymdhms, "%d/%d/%d %d:%d:%lf", &start_year_, &start_month_, &start_day_, &start_hour_, &start_minute_, &start_sec_);
  jday(start_year_, start_month_, start_day_, start_hour_, start_minute_, start_sec_, start_jd_);
  current_jd_ = start_jd_;
  current_sidereal_ = gstime(current_jd_);
  JdToDecyear(current_jd_, &current_decyear_);
  ConvJDtoCalendarDay(current_jd_);
  AssertTimeStepParams();
  InitializeState();
  SetParameters();

  // Ephemeris time initialize
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(11) << "jd " << start_jd_;
  str2et_c(stream.str().c_str(), &start_ephemeris_time_);
}

SimulationTime::~SimulationTime() {}

void SimulationTime::AssertTimeStepParams() {
  // Runge-Kutta time step must be smaller than its update interval
  assert(attitude_rk_step_sec_ <= attitude_update_interval_sec_);
  assert(orbit_rk_step_sec_ <= orbit_update_interval_sec_);
  assert(thermal_rk_step_sec_ <= thermal_update_interval_sec_);

  // Step time for the entire simulation must be smaller than all of the subroutine step times
  assert(step_sec_ <= attitude_update_interval_sec_);
  assert(step_sec_ <= orbit_update_interval_sec_);
  assert(step_sec_ <= thermal_update_interval_sec_);
  assert(step_sec_ <= component_update_interval_sec_);
  assert(step_sec_ <= log_output_interval_sec_);
}

void SimulationTime::SetParameters(void) {
  elapsed_time_sec_ = 0.0;
  attitude_update_counter_ = 1;
  attitude_update_flag_ = false;
  orbit_update_counter_ = 1;
  orbit_update_flag_ = false;
  thermal_update_counter_ = 1;
  thermal_update_flag_ = false;
  component_update_counter_ = 1;
  component_update_flag_ = false;
  log_counter_ = 0;
  display_counter_ = 0;
  state_.log_output = true;
}

void SimulationTime::UpdateTime(void) {
  InitializeState();
  elapsed_time_sec_ += step_sec_;
  if (simulation_speed_ > 0) {
    chrono::system_clock clk;
    int toWaitTime = (int)(elapsed_time_sec_ * 1000 -
                           chrono::duration_cast<chrono::milliseconds>(clk.now() - clock_start_time_millisec_).count() * simulation_speed_);
    if (toWaitTime <= 0) {
      // When the execution time is larger than specified step_sec

      int exceeded_duration_ms = (int)chrono::duration_cast<chrono::milliseconds>(clk.now() - clock_last_time_completed_step_in_time_).count();
      if (exceeded_duration_ms > time_exceeds_continuously_limit_sec_ * 1000) {
        // Skip time and warn only when execution time exceeds continuously for long time

        cout << "Error: the specified step_sec is too small for this computer.\r\n";

        // Forcibly set elapsed_tim_sec_ as actual elapsed time Reason: to catch up with real time when resume from a breakpoint
        elapsed_time_sec_ =
            (chrono::duration_cast<chrono::duration<double, ratio<1, 1>>>(clk.now() - clock_start_time_millisec_).count() * simulation_speed_);

        clock_last_time_completed_step_in_time_ = clk.now();
      }
    } else {
      clock_last_time_completed_step_in_time_ = clk.now();

#ifdef WIN32
      Sleep(toWaitTime);
#else
      const struct timespec req = {toWaitTime / 1000, (toWaitTime % 1000) * 1000000};
      nanosleep(&req, NULL);
#endif
    }
  }

  attitude_update_counter_++;
  orbit_update_counter_++;
  thermal_update_counter_++;
  component_update_counter_++;
  log_counter_++;
  display_counter_++;

  if (elapsed_time_sec_ > end_sec_) {
    state_.finish = true;
  }

  current_jd_ = start_jd_ + elapsed_time_sec_ / (60.0 * 60.0 * 24.0);
  current_sidereal_ = gstime(current_jd_);
  JdToDecyear(current_jd_, &current_decyear_);
  ConvJDtoCalendarDay(current_jd_);

  attitude_update_flag_ = false;
  if (double(attitude_update_counter_) * step_sec_ >= attitude_update_interval_sec_) {
    attitude_update_counter_ = 0;
    attitude_update_flag_ = true;
  }

  orbit_update_flag_ = false;
  if (double(orbit_update_counter_) * step_sec_ >= orbit_update_interval_sec_) {
    orbit_update_counter_ = 0;
    orbit_update_flag_ = true;
  }

  thermal_update_flag_ = false;
  if (double(thermal_update_counter_) * step_sec_ >= thermal_update_interval_sec_) {
    thermal_update_counter_ = 0;
    thermal_update_flag_ = true;
  }

  component_update_flag_ = false;
  if (double(component_update_counter_) * step_sec_ >= component_update_interval_sec_) {
    component_update_counter_ = 0;
    component_update_flag_ = true;
  }

  if (double(log_counter_) * step_sec_ >= log_output_interval_sec_) {
    log_counter_ = 0;
    state_.log_output = true;
  }

  if (display_counter_ >= display_period_) {
    display_counter_ -= (int)display_period_;
    state_.disp_output = true;
  }

  state_.running = true;
}

void SimulationTime::ResetClock(void) { clock_start_time_millisec_ = chrono::system_clock::now(); }

void SimulationTime::PrintStartDateTime(void) const {
  int sec_int = int(start_sec_ + 0.5);
  stringstream s, m, h;
  if (sec_int < 10) {
    s << 0 << sec_int;
  } else {
    s << sec_int;
  }
  if (start_minute_ < 10) {
    m << 0 << start_minute_;
  } else {
    m << start_minute_;
  }
  if (start_hour_ < 10) {
    h << 0 << start_hour_;
  } else {
    h << start_hour_;
  }

  cout << " " << start_year_ << "/" << start_month_ << "/" << start_day_ << " " << h.str() << ":" << m.str() << ":" << s.str() << "\n";
}

string SimulationTime::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteScalar("elapsed_time", "s");
  str_tmp += WriteScalar("time", "UTC");

  return str_tmp;
}

string SimulationTime::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteScalar(elapsed_time_sec_);

  const char kSize = 100;
  char ymdhms[kSize];
  double sec_floor = floor(current_utc_.second * 1e3) / 1e3;

  snprintf(ymdhms, kSize, "%4d/%02d/%02d %02d:%02d:%.3f,", current_utc_.year, current_utc_.month, current_utc_.day, current_utc_.hour,
           current_utc_.minute, sec_floor);
  str_tmp += ymdhms;

  return str_tmp;
}

void SimulationTime::InitializeState() {
  state_.disp_output = false;
  state_.finish = false;
  state_.log_output = false;
  state_.running = false;
}

// wrapper function of invjday @ sgp4ext for interface adjustment
void SimulationTime::ConvJDtoCalendarDay(const double JD) {
  int year, mon, day, hr, minute;
  double sec;
  invjday(JD, year, mon, day, hr, minute, sec);
  current_utc_.year = (unsigned int)(year);
  current_utc_.month = (unsigned int)(mon);
  current_utc_.day = (unsigned int)(day);
  current_utc_.hour = (unsigned int)(hr);
  current_utc_.minute = (unsigned int)(minute);
  current_utc_.second = sec;
}

SimulationTime* InitSimulationTime(std::string file_name) {
  setting_file_reader::IniAccess ini_file(file_name);

  const char* section = "TIME";
  // Parameters about entire simulation
  std::string start_ymdhms = ini_file.ReadString(section, "simulation_start_time_utc");
  double end_sec = ini_file.ReadDouble(section, "simulation_duration_s");
  double step_sec = ini_file.ReadDouble(section, "simulation_step_s");

  // Time step parameters for dynamics propagation
  double attitude_update_interval_sec = ini_file.ReadDouble(section, "attitude_update_period_s");
  double attitude_rk_step_sec = ini_file.ReadDouble(section, "attitude_integral_step_s");
  double orbit_update_interval_sec = ini_file.ReadDouble(section, "orbit_update_period_s");
  double orbit_rk_step_sec = ini_file.ReadDouble(section, "orbit_integral_step_s");
  double thermal_update_interval_sec = ini_file.ReadDouble(section, "thermal_update_period_s");
  double thermal_rk_step_sec = ini_file.ReadDouble(section, "thermal_integral_step_s");

  // Time step parameter for component propagation
  double compo_propagate_step_sec = ini_file.ReadDouble(section, "component_update_period_s");

  // Time step parameter for log output
  double log_output_interval_sec = ini_file.ReadDouble(section, "log_output_period_s");

  double sim_speed = ini_file.ReadDouble(section, "simulation_speed_setting");

  SimulationTime* simTime = new SimulationTime(end_sec, step_sec, attitude_update_interval_sec, attitude_rk_step_sec, orbit_update_interval_sec,
                                               orbit_rk_step_sec, thermal_update_interval_sec, thermal_rk_step_sec, compo_propagate_step_sec,
                                               log_output_interval_sec, start_ymdhms.c_str(), sim_speed);

  return simTime;
}

} // namespace s2e::environment
