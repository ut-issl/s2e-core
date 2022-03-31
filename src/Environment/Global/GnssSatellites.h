#ifndef __gnss_satellites_h__
#define __gnss_satellites_h__

#include <Interface/LogOutput/ILoggable.h>

#include <Library/math/Vector.hpp>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <vector>

#include "SimTime.h"

extern const double speed_of_light;  //[m/s] in vacuum
extern const double nan99;

#define ECEF 0
#define ECI 1

#define Lagrange 0
#define Trigonometric 1

// #define GNSS_SATELLITES_DEBUG_OUTPUT // for debug output, Uncomment

/**
 * @enum UR_KINDS
 * @enum When Using Ultra Rapid Calendar, decide to use which 6 hours in each
 * observe and predict 24 hours calender
 */
typedef enum {
  UR_NOT_UR,  //!< don't use UR

  UR_OBSERVE1,  //!< the most oldest observe 6 hours (most precise)
  UR_OBSERVE2,  //!< the second oldest observe 6 hours (6 ~ 12)
  UR_OBSERVE3,
  UR_OBSERVE4,

  UR_PREDICT1,  //!< the most oldest presercve 6 hours (most precise)
  UR_PREDICT2,
  UR_PREDICT3,
  UR_PREDICT4,

  UR_UNKNOWN
} UR_KINDS;

class GnssSat_coordinate {
 public:
  int GetIndexFromID(std::string sat_num) const;
  std::string GetIDFromIndex(int index) const;
  int GetNumOfSatellites() const;
  bool GetWhetherValid(int sat_id) const;

 protected:
  /*
  https://en.wikipedia.org/wiki/Trigonometric_interpolation#
  http://acc.igs.org/orbits/orbit-interp_gpssoln03.pdf
  */
  template <size_t N>
  libra::Vector<N> TrigonometricInterpolation(const std::vector<double>& time_vector, const std::vector<libra::Vector<N>>& values, double time) const;
  double TrigonometricInterpolation(const std::vector<double>& time_vector, const std::vector<double>& values, double time) const;
  template <size_t N>
  libra::Vector<N> LagrangeInterpolation(const std::vector<double>& time_vector, const std::vector<libra::Vector<N>>& values, double time) const;
  double LagrangeInterpolation(const std::vector<double>& time_vector, const std::vector<double>& values, double time) const;

  std::vector<std::vector<double>> unixtime_vector_;  // unixtime for all sat

  std::vector<std::vector<double>> time_period_;  // for inter polation

  std::vector<bool> validate_;      // whether available at the time
  std::vector<int> nearest_index_;  // index list for update(in position,
                                    // time_and_index_list_. in clock_bias, time_table_)

  double step_sec_;
  double time_interval_;
  int interpolation_number_;
};

class GnssSat_position : public GnssSat_coordinate {
 public:
  GnssSat_position() {}
  std::pair<double, double> Init(std::vector<std::vector<std::string>>& file, int interpolation_method, int interpolation_number, UR_KINDS ur_flag);
  void SetUp(const double start_unix_time, const double step_sec);
  void Update(const double now_unix_time);

  libra::Vector<3> GetSatEcef(int sat_id) const;
  libra::Vector<3> GetSatEci(int sat_id) const;

 private:
  // gnss_satellites_position [m]
  std::vector<libra::Vector<3>> gnss_sat_ecef_;
  std::vector<libra::Vector<3>> gnss_sat_eci_;

  std::vector<std::vector<libra::Vector<3>>> gnss_sat_table_ecef_;
  std::vector<std::vector<libra::Vector<3>>> gnss_sat_table_eci_;

  std::vector<std::vector<libra::Vector<3>>> ecef_;
  std::vector<std::vector<libra::Vector<3>>> eci_;
};

class GnssSat_clock : public GnssSat_coordinate {
 public:
  GnssSat_clock() {}
  void Init(std::vector<std::vector<std::string>>& file, std::string file_extension, int interpolation_number, UR_KINDS ur_flag,
            std::pair<double, double> unix_time_period);
  void SetUp(const double start_unix_time, const double step_sec);
  void Update(const double now_unix_time);

  double GetSatClock(int sat_id) const;

 private:
  std::vector<double> gnss_sat_clock_;  // clock bias [m], not micro second
  std::vector<std::vector<double>> gnss_sat_clock_table_;
  std::vector<std::vector<double>> clock_bias_;
};

class GnssSat_Info {
 public:
  GnssSat_Info();
  void Init(std::vector<std::vector<std::string>>& position_file, int position_interpolation_method, int position_interpolation_number,
            UR_KINDS position_ur_flag,

            std::vector<std::vector<std::string>>& clock_file, std::string clock_file_extension, int clock_interpolation_number,
            UR_KINDS clock_ur_flag);
  void SetUp(const double start_unix_time, const double step_sec);
  void Update(const double now_unix_time);

  int GetNumOfSatellites() const;
  bool GetWhetherValid(int sat_id) const;
  libra::Vector<3> GetSatellitePositionEcef(int sat_id) const;
  libra::Vector<3> GetSatellitePositionEci(int sat_id) const;
  double GetSatelliteClock(int sat_id) const;
  const GnssSat_position& GetGnssSatPos() const;
  const GnssSat_clock& GetGnssSatClock() const;

 private:
  GnssSat_position position_;
  GnssSat_clock clock_;
};

class GnssSatellites : public ILoggable {
 public:
  // CONSTRUCTOR AND DECONSTRUCTOR
  GnssSatellites(bool is_calc_enabled);
  virtual ~GnssSatellites() {}

  void Init(std::vector<std::vector<std::string>>& true_position_file, int true_position_interpolation_method, int true_position_interpolation_number,
            UR_KINDS true_position_ur_flag,

            std::vector<std::vector<std::string>>& true_clock_file, std::string true_clock_file_extension, int true_clock_interpolation_number,
            UR_KINDS true_clock_ur_flag,

            std::vector<std::vector<std::string>>& estimate_position_file, int estimate_position_interpolation_method,
            int estimate_position_interpolation_number, UR_KINDS estimate_position_ur_flag,

            std::vector<std::vector<std::string>>& estimate_clock_file, std::string estimate_clock_file_extension,
            int estimate_clock_interpolation_number, UR_KINDS estimate_clock_ur_flag);

  bool IsCalcEnabled() const;

  void SetUp(const SimTime* sim_time);
  void Update(const SimTime* sim_time);

  // GET FUNCTIONS
  int GetIndexFromID(std::string sat_num) const;
  std::string GetIDFromIndex(int index) const;
  int GetNumOfSatellites() const;
  bool GetWhetherValid(int sat_id) const;
  double GetStartUnixTime() const;
  const GnssSat_Info& Get_true_info() const;
  const GnssSat_Info& Get_estimate_info() const;

  libra::Vector<3> GetSatellitePositionEcef(const int sat_id) const;
  libra::Vector<3> GetSatellitePositionEci(const int sat_id) const;

  double GetSatelliteClock(const int sat_id) const;

  // Get observation Range:[m] CarrierPhase:[no unit], frequency is thought to
  // be expressed in MHz
  double GetPseudoRangeECEF(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;
  double GetPseudoRangeECI(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;
  std::pair<double, double> GetCarrierPhaseECEF(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;
  std::pair<double, double> GetCarrierPhaseECI(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;

  // FOR LOG OUTPUT // 継承のために
  std::string GetLogHeader() const override;
  std::string GetLogValue() const override;

  // FOR DEBUG OUTPUT
  void DebugOutput(void);

 private:
  double TrigonometricInterpolation(std::vector<double> time_period, std::vector<double> position, double time);
  // for Ionospheric delay very Miscellaneous need to fix.
  double AddIonosphericDelay(const int sat_id, const libra::Vector<3> rec_position, const double frequency, const bool flag) const;

  bool is_calc_enabled_ = true;
  GnssSat_Info true_info_;
  GnssSat_Info estimate_info_;
  double start_unix_time_;

#ifdef GNSS_SATELLITES_DEBUG_OUTPUT
  ofstream ofs_true;
  ofstream ofs_esti;
  ofstream ofs_sa;
#endif
};
#endif
