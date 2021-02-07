#ifndef __gnss_satellites_h__
#define __gnss_satellites_h__

#include <cmath>
#include <vector>
#include <map>
#include <ctime>
#include <fstream>
#include <iomanip>

#include "SimTime.h"
#include "../../Library/math/Vector.hpp"
#include "../../Interface/LogOutput/ILoggable.h"

const double speed_of_light = 299792458.0; //[m/s] in vacuum

#define ECEF 0
#define ECI 1

#define Lagrange 0
#define Trigonometric 1

const double nan99 = 999999.999999;

class GnssSat_coordinate
{
  public:
    int GetIndexFromID(string sat_num) const;
    string GetIDFromIndex(int index) const;
    int GetNumOfSatellites() const;
    bool GetWhetherValid(int sat_id) const;

  protected:
    void ProcessData(vector<vector<string>>& raw_file, vector<pair<string, vector<string>>>& data);
    void ProcessData(vector<vector<string>>& raw_file, vector<pair<string, vector<string>>>& data, bool ur_flag);
    void ProcessData(vector<vector<string>>& raw_file, vector<string>& data);
    /*
    https://en.wikipedia.org/wiki/Trigonometric_interpolation#
    http://acc.igs.org/orbits/orbit-interp_gpssoln03.pdf
    */
    template<size_t N> Vector<N> TrigonometricInterpolation(const vector<double>& time_vector, const vector<Vector<N>>& values, double time) const;
    double TrigonometricInterpolation(const vector<double>& time_vector, const vector<double>& values, double time) const;
    template<size_t N> Vector<N> LagrangeInterpolation(const vector<double>& time_vector, const vector<Vector<N>>& values, double time) const;
    double LagrangeInterpolation(const vector<double>& time_vector, const vector<double>& values, double time) const;

    std::vector<double> time_table_; //unix_time
    std::vector<std::vector<bool>> available_table_; //whether available for every time
    std::vector<bool> validate_; //whether available at the time
    std::vector<int> nearest_index_; //index list for update(in position, time_and_index_list_. in clock_bias, time_table_)
    std::vector<std::vector<double>> time_vector_;

    double step_sec_;
    double time_interval_ = 1e9;
    int interpolation_number_;

    const int gps_sat_num_ = 32;
    const int glonass_sat_num_ = 26;
    const int galileo_sat_num_ = 36;
    const int beidou_sat_num_ = 16;
    const int qzss_sat_num_ = 7;

    const int gps_index_bias_ = -1;
    const int glonass_index_bias_ = gps_index_bias_ + gps_sat_num_;
    const int galileo_index_bias_ = glonass_index_bias_ + glonass_sat_num_;
    const int beidou_index_bias_ = galileo_index_bias_ + galileo_sat_num_;
    const int qzss_index_bias_ = beidou_index_bias_ + beidou_sat_num_;

    const int all_sat_num_ = gps_sat_num_ + glonass_sat_num_ + galileo_sat_num_ + beidou_sat_num_ + qzss_sat_num_;
};

class GnssSat_position: public GnssSat_coordinate
{
  public:
    GnssSat_position() {}
    void Init(vector<vector<string>>& file, int interpolation_method,
              int interpolation_number, bool ur_flag);
    void SetUp(const double start_unix_time, const double step_sec);
    void Update(const double now_unix_time);

    Vector<3> GetSatEcef(int sat_id) const;
    Vector<3> GetSatEci(int sat_id) const;
  
  private:
    //gnss_satellites_position [m]
    std::vector<libra::Vector<3>> gnss_sat_ecef_;
    std::vector<libra::Vector<3>> gnss_sat_eci_;

    std::vector<std::vector<libra::Vector<3>>> gnss_sat_table_ecef_;
    std::vector<std::vector<libra::Vector<3>>> gnss_sat_table_eci_;
    vector<vector<pair<double, int>>> time_and_index_list_; //contains the pair of available time and index.

    vector<vector<Vector<3>>> ecef_;
    vector<vector<Vector<3>>> eci_;
};

class GnssSat_clock: public GnssSat_coordinate
{
  public:
    GnssSat_clock() {}
    void Init(vector<vector<string>>& file, string file_extension,
              int interpolation_number, bool ur_flag);
    void SetUp(const double start_unix_time, const double step_sec);
    void Update(const double now_unix_time);

    double GetSatClock(int sat_id) const;

  private:
    std::vector<double> gnss_sat_clock_; //clock bias [m], not micro second
    std::vector<std::vector<double>> gnss_sat_clock_table_;
    vector<int> time_and_index_num_; //clock bias calculation should be stict, so it includes time_table_index(not for available)
    std::vector<std::vector<double>> clock_bias_;
};

class GnssSat_Info
{
  public:
    GnssSat_Info();
    void Init(vector<vector<string>>& position_file,
              int position_interpolation_method,
              int position_interpolation_number,
              bool position_ur_flag,

              vector<vector<string>>& clock_file,
              string clock_file_extension,
              int clock_interpolation_number,
              bool clock_ur_flag);
    void SetUp(const double start_unix_time, const double step_sec);
    void Update(const double now_unix_time);

    int GetNumOfSatellites() const;
    bool GetWhetherValid(int sat_id) const;
    libra::Vector<3> GetSatellitePositionEcef(int sat_id) const;
    libra::Vector<3> GetSatellitePositionEci(int sat_id) const;
    double GetSatelliteClock(int sat_id) const;
    const  GnssSat_position& GetGnssSatPos() const;
    const  GnssSat_clock& GetGnssSatClock() const;

  private:
    GnssSat_position position_;
    GnssSat_clock clock_;
};

class GnssSatellites : public ILoggable
{
public:
  //CONSTRUCTOR AND DECONSTRUCTOR
  GnssSatellites(bool is_calc_enabled);
  ~GnssSatellites() {}

  void Init(vector<vector<string>>& true_position_file,
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
            bool estimate_clock_ur_flag);

  bool IsCalcEnabled() const;

  void SetUp(const SimTime* sim_time);
  void Update(const SimTime* sim_time);

  //GET FUNCTIONS
  int GetIndexFromID(string sat_num) const;
  string GetIDFromIndex(int index) const;
  int GetNumOfSatellites() const;
  bool GetWhetherValid(int sat_id) const;
  double GetStartUnixTime() const;
  const GnssSat_Info& Get_true_info() const;
  const GnssSat_Info& Get_estimate_info() const;

  libra::Vector<3> GetSatellitePositionEcef(const int sat_id) const;
  libra::Vector<3> GetSatellitePositionEci(const int sat_id) const;

  double GetSatelliteClock(const int sat_id) const;

  //Get observation Range:[m] CarrierPhase:[no unit]
  double GetPseudoRangeECEF(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;
  double GetPseudoRangeECI(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;
  pair<double, double> GetCarrierPhaseECEF(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;
  pair<double, double> GetCarrierPhaseECI(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;

  // FOR LOG OUTPUT // 継承のために
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

  // FOR DEBUG OUTPUT
  void DebugOutput(void);

private:
  double TrigonometricInterpolation(vector<double> time_period, vector<double> position, double time);
  // for Ionospheric delay very Miscellaneous need to fix.
  double AddIonosphericDelay(const int sat_id, const libra::Vector<3> rec_position, const double frequency, const bool flag) const;

  bool is_calc_enabled_ = true;
  GnssSat_Info true_info_;
  GnssSat_Info estimate_info_;
  double start_unix_time_;
  const double clock_bias_bias_ = 0.0; //there is fixed bias between IGS clk and other files...(I don't know why)

  //ofstream ofs_true;
  //ofstream ofs_esti;
  //ofstream ofs_sa;
};
#endif
