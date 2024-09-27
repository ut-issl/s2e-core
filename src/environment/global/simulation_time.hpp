/**
 *@file simulation_time.hpp
 *@brief Class to manage simulation time related information
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_SIMULATION_TIME_HPP_
#define S2E_ENVIRONMENT_GLOBAL_SIMULATION_TIME_HPP_

#ifdef WIN32
#define _WINSOCKAPI_  // stops windows.h including winsock.h
#endif

#include <string>
// #include <time.h>
#include <chrono>

#include "logger/loggable.hpp"
#include "math_physics/orbit/sgp4/sgp4ext.h"
#include "math_physics/orbit/sgp4/sgp4io.h"
#include "math_physics/orbit/sgp4/sgp4unit.h"

namespace s2e::environment {

/**
 *@struct TimeState
 *@brief State of timing controller
 */
struct TimeState {
  bool running = false;
  bool finish = false;
  bool log_output = false;
  bool disp_output = true;
};

/**
 *@struct UTC
 *@brief UTC (Coordinated Universal Time) calendar expression
 */
struct UTC {
  unsigned int year = 2000;
  unsigned int month = 1;
  unsigned int day = 1;
  unsigned int hour = 0;
  unsigned int minute = 0;
  double second = 0.0;
};

/**
 *@class SimulationTime
 *@brief Class to manage simulation time related information
 */
class SimulationTime : public logger::ILoggable {
 public:
  /**
   *@fn SimulationTime
   *@brief Constructor
   *@param [in] end_sec: Simulation duration [sec]
   *@param [in] step_sec: Simulation step [sec]
   *@param [in] attitude_update_interval_sec: Attitude update interval [sec]
   *@param [in] attitude_rk_step_sec: Attitude Runge-Kutta step time [sec]
   *@param [in] orbit_update_interval_sec: Orbit update interval [sec]
   *@param [in] orbit_rk_step_sec: Orbit Runge-Kutta step time [sec]
   *@param [in] thermal_update_interval_sec: Thermal update interval [sec]
   *@param [in] thermal_rk_step_sec: Thermal Runge-Kutta step time [sec]
   *@param [in] compo_propagate_step_sec: Component propagate step time [sec]
   *@param [in] log_output_interval_sec: Log output interval [sec]
   *@param [in] start_ymdhms: Simulation start time in UTC [YYYYMMDD hh:mm:ss]
   *@param [in] sim_speed: Simulation speed setting
   */
  SimulationTime(const double end_sec, const double step_sec, const double attitude_update_interval_sec, const double attitude_rk_step_sec,
                 const double orbit_update_interval_sec, const double orbit_rk_step_sec, const double thermal_update_interval_sec,
                 const double thermal_rk_step_sec, const double compo_propagate_step_sec, const double log_output_interval_sec,
                 const char* start_ymdhms, const double sim_speed);
  /**
   *@fn ~SimulationTime
   *@brief Destructor
   */
  virtual ~SimulationTime();

  /**
   *@fn SetParameters
   *@brief Set time related parameters for initializing simulation
   *@note This function should be called all cases in the Monte-Carlo simulation
   */
  void SetParameters(void);
  /**
   *@fn UpdateTime
   *@brief Update simulation time
   */
  void UpdateTime(void);
  /**
   *@fn ResetClock
   *@brief Reset simulation start time as PC’s time
   */
  void ResetClock(void);

  /**
   *@fn GetState
   *@brief Return time state
   */
  inline const TimeState GetState(void) const { return state_; };
  /**
   *@fn GetElapsedTime_s
   *@brief Return simulation elapsed time [sec]
   */
  inline double GetElapsedTime_s(void) const { return elapsed_time_sec_; };
  /**
   *@fn GetSimulationStep_s
   *@brief Return simulation step [sec]
   */
  inline double GetSimulationStep_s(void) const { return step_sec_; };
  /**
   *@fn GetAttitudeUpdateInterval_s
   *@brief Return attitude update interval [sec]
   */
  inline double GetAttitudeUpdateInterval_s(void) const { return attitude_update_interval_sec_; };
  /**
   *@fn GetAttitudePropagateFlag
   *@brief Return attitude propagate flag
   */
  inline bool GetAttitudePropagateFlag(void) const { return attitude_update_flag_; };
  /**
   *@fn GetAttitudeRkStepTime_s
   *@brief Return attitude Runge-Kutta step time [sec]
   */
  inline double GetAttitudeRkStepTime_s() const { return attitude_rk_step_sec_; }

  /**
   *@fn GetOrbitUpdateInterval_s
   *@brief Return orbit update interval [sec]
   */
  inline double GetOrbitUpdateInterval_s(void) const { return orbit_update_interval_sec_; };
  /**
   *@fn GetOrbitPropagateFlag
   *@brief Return orbit propagate flag
   */
  inline bool GetOrbitPropagateFlag(void) const { return orbit_update_flag_; };
  /**
   *@fn GetOrbitRkStepTime_s
   *@brief Return orbit Runge-Kutta step time [sec]
   */
  inline double GetOrbitRkStepTime_s() const { return orbit_rk_step_sec_; }

  /**
   *@fn GetThermalUpdateIntervalSec
   *@brief Return thermal update interval [sec]
   */
  inline double GetThermalUpdateInterval_s(void) const { return thermal_update_interval_sec_; };
  /**
   *@fn GetThermalPropagateFlag
   *@brief Return thermal propagate flag
   */
  inline bool GetThermalPropagateFlag(void) const { return thermal_update_flag_; };
  /**
   *@fn GetThermalRkStepTime_s
   *@brief Return thermal Runge-Kutta step time [sec]
   */
  inline double GetThermalRkStepTime_s() const { return thermal_rk_step_sec_; }

  /**
   *@fn GetComponentStepTime_s
   *@brief Return component update step time [sec]
   */
  inline double GetComponentStepTime_s(void) const { return component_update_interval_sec_; };
  /**
   *@fn GetCompoUpdateFlag
   *@brief Return component update flag
   */
  inline bool GetCompoUpdateFlag() const { return component_update_flag_; }
  /**
   *@fn GetComponentPropagateFrequency_Hz
   *@brief Return component propagate frequency [Hz]
   */
  inline int GetComponentPropagateFrequency_Hz(void) const { return component_propagate_frequency_Hz_; };

  /**
   *@fn GetEndTime_s
   *@brief Return simulation end elapsed time [sec]
   */
  inline double GetEndTime_s(void) const { return end_sec_; };
  /**
   *@fn GetProgressionRate
   *@brief Return progression rate of the simulation [%]
   */
  inline int GetProgressionRate(void) const { return (int)floor((elapsed_time_sec_ / end_sec_ * 100)); };

  /**
   *@fn GetCurrentTime_jd
   *@brief Return current Julian day [day]
   */
  inline double GetCurrentTime_jd(void) const { return current_jd_; };
  /**
   *@fn GetCurrentSiderealTime
   *@brief Return current sidereal day [day]
   */
  inline double GetCurrentSiderealTime(void) const { return current_sidereal_; };
  /**
   *@fn GetCurrentDecimalYear
   *@brief Return current decimal year [year]
   */
  inline double GetCurrentDecimalYear(void) const { return current_decyear_; };
  /**
   *@fn GetCurrentUtc
   *@brief Return current UTC calendar expression
   */
  inline const UTC GetCurrentUtc(void) const { return current_utc_; };
  /**
   *@fn GetCurrentEphemerisTime
   *@brief Return current Ephemeris time
   */
  inline double GetCurrentEphemerisTime(void) const { return start_ephemeris_time_ + elapsed_time_sec_; };

  /**
   *@fn GetStartYear
   *@brief Return start time year [year]
   */
  inline int GetStartYear(void) const { return start_year_; };
  /**
   *@fn GetStartMonth
   *@brief Return start time month [month]
   */
  inline int GetStartMonth(void) const { return start_month_; };
  /**
   *@fn GetStartDay
   *@brief Return start time day [day]
   */
  inline int GetStartDay(void) const { return start_day_; };
  /**
   *@fn GetStartHour
   *@brief Return start time hour [hour]
   */
  inline int GetStartHour(void) const { return start_hour_; };
  /**
   *@fn GetStartMinute
   *@brief Return start time minute [minute]
   */
  inline int GetStartMinute(void) const { return start_minute_; };
  /**
   *@fn GetStartSecond
   *@brief Return start time second [sec]
   */
  inline double GetStartSecond(void) const { return start_sec_; };

  // Override logger::ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of logger::ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of logger::ILoggable
   */
  virtual std::string GetLogValue() const;

  /**
   * @fn PrintStartDateTime
   * @brief Debug output of start date and time
   */
  void PrintStartDateTime(void) const;

 private:
  // Variables
  double elapsed_time_sec_;  //!< Elapsed time from start of simulation [sec]
  double current_jd_;        //!< Current Julian date [day]
  double current_sidereal_;  //!< Current Greenwich sidereal time (GST) [day]
  double current_decyear_;   //!< Current decimal year [year]
  UTC current_utc_;          //!< UTC calendar day

  // Timing controller
  int attitude_update_counter_;   //!< Update counter for attitude calculation
  bool attitude_update_flag_;     //!< Update flag for attitude calculation
  int orbit_update_counter_;      //!< Update counter for orbit calculation
  bool orbit_update_flag_;        //!< Update flag for orbit calculation
  int thermal_update_counter_;    //!< Update counter for thermal calculation
  bool thermal_update_flag_;      //!< Update flag for thermal calculation
  int component_update_counter_;  //!< Update counter for component calculation
  bool component_update_flag_;    //!< Update flag for component calculation
  int log_counter_;               //!< Update counter for log output
  int display_counter_;           //!< Update counter for display output
  TimeState state_;               //!< State of timing controller

  // Calculation time measure
  std::chrono::system_clock::time_point clock_start_time_millisec_;  //!< Simulation start time [ms]
  // chrono::system_clock::time_point clock_elapsed_time_millisec_;  //!< Simulation elapsed time in real time simulation [ms]
  std::chrono::system_clock::time_point clock_last_time_completed_step_in_time_;  //!< Simulation finished time [ms]

  // Constants
  double end_sec_;                        //!< Time from start of simulation to end [sec]
  double step_sec_;                       //!< Simulation step width [sec]
  double attitude_update_interval_sec_;   //!< Update intercal for attitude calculation [sec]
  double attitude_rk_step_sec_;           //!< Runge-Kutta step width for attitude calculation [sec]
  double orbit_update_interval_sec_;      //!< Update intercal for orbit calculation [sec]
  double orbit_rk_step_sec_;              //!< Runge-Kutta step width for orbit calculation [sec]
  double thermal_update_interval_sec_;    //!< Update intercal for thermal calculation [sec]
  double thermal_rk_step_sec_;            //!< Runge-Kutta step width for thermal calculation [sec]
  double component_update_interval_sec_;  //!< Update intercal for component calculation [sec]
  int component_propagate_frequency_Hz_;  //!< Component propagation frequency [Hz]
  double log_output_interval_sec_;        //!< Log output interval [sec]
  double display_period_;                 //!< Display output period [sec]

  double start_ephemeris_time_;  //!< Simulation start Ephemeris Time
  double start_jd_;              //!< Simulation start Julian date [day]
  int start_year_;               //!< Simulation start year
  int start_month_;              //!< Simulation start month
  int start_day_;                //!< Simulation start day
  int start_hour_;               //!< Simulation start hour
  int start_minute_;             //!< Simulation start minute
  double start_sec_;             //!< Simulation start seconds

  double simulation_speed_;  //!< The speed of the simulation relative to real time (if negative, real time is not taken into account)
  double time_exceeds_continuously_limit_sec_;  //!< Maximum duration to allow actual step_sec to be larger than specified continuously

  /**
   * @fn InitializeState
   * @brief Initialize timer state
   */
  void InitializeState();
  /**
   * @fn AssertTimeStepParams
   * @brief Check the timing setting parameters are correct
   */
  void AssertTimeStepParams();
  /**
   * @fn ConvJDtoCalendarDay
   * @brief Convert Julian date to UTC Calendar date
   * @note wrapper function of invjday @ sgp4ext for interface adjustment
   */
  void ConvJDtoCalendarDay(const double JD);
};

/**
 *@fn InitSimulationTime
 *@brief Initialize function for SimulationTime class
 *@param [in] file_name: Path to the initialize function
 */
SimulationTime* InitSimulationTime(std::string file_name);

} // namespace s2e::environment

#endif  // S2E_ENVIRONMENT_GLOBAL_SIMULATION_TIME_HPP_
