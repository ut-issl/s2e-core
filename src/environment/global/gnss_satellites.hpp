/**
 * @file gnss_satellites.hpp
 * @brief Class to calculate GNSS satellite position and related states
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_GNSS_SATELLITES_H_
#define S2E_ENVIRONMENT_GLOBAL_GNSS_SATELLITES_H_

#include <Interface/LogOutput/ILoggable.h>

#include <Library/math/Vector.hpp>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <vector>

#include "SimTime.h"

extern const double nan99;  //!< Not at Number TODO: Should be moved to another place

// TODO: Use enum
#define ECEF 0  //!< Use ECEF frame for GNSS satellite position frame in Add_IonosphericDelay
#define ECI 1   //!< Use ECI frame for GNSS satellite position frame in Add_IonosphericDelay

// TODO: Not used now. Remove? Use enum?
#define Lagrange 0       //!< Use Lagrange interpolation
#define Trigonometric 1  //!< Use Trigonometric interpolation

// #define GNSS_SATELLITES_DEBUG_OUTPUT //!< For debug output, uncomment this

/**
 * @enum UR_KINDS
 * @brief Ultra Rapid mode
 * @details When Using Ultra Rapid ephemerides, decide to use which 6 hours in each observe and predict 24 hours
 */
typedef enum {
  UR_NOT_UR,  //!< Don't use ultra rapid

  UR_OBSERVE1,  //!< the most oldest observe 6 hours (most precise)
  UR_OBSERVE2,  //!< the second oldest observe 6 hours (6 ~ 12)
  UR_OBSERVE3,
  UR_OBSERVE4,

  UR_PREDICT1,  //!< the most oldest preserve 6 hours (most precise)
  UR_PREDICT2,
  UR_PREDICT3,
  UR_PREDICT4,

  UR_UNKNOWN
} UR_KINDS;

/**
 * @class GnssSat_coordinate
 * @brief GNSS satellite coordinate?
 */
class GnssSat_coordinate {
 public:
  /**
   * @fn GetIndexFromID
   * @brief Calculate index of GNSS satellite defined in this class from GNSS satellite number defined in GNSS system
   * @return Index of GNSS satellite defined in this class
   */
  int GetIndexFromID(std::string sat_num) const;
  /**
   * @fn GetIDFromIndex
   * @brief Calculate GNSS satellite number defined in GNSS system from index of GNSS satellite defined in this class
   * @return GNSS satellite number defined in GNSS system
   */
  std::string GetIDFromIndex(int index) const;
  /**
   * @fn GetNumOfSatellites
   * @brief Return total satellite number in all GNSS system (Constant value)
   */
  int GetNumOfSatellites() const;
  /**
   * @fn GetWhetherValid
   * @brief Return true the GNSS satellite information is available
   * @param [in] sat_id: Index of GNSS satellite
   */
  bool GetWhetherValid(int sat_id) const;

 protected:
  /**
   * @fn TrigonometricInterpolation
   * @brief Interpolate with Trigonometric method
   * @note Ref: http://acc.igs.org/orbits/orbit-interp_gpssoln03.pdf
   *            https://en.wikipedia.org/wiki/Trigonometric_interpolation#
   * @param [in] time_vector: List of given time
   * @param [in] values: List of given value
   * @param [in] time: Time to calculate the interpolated value
   * @return Interpolated value
   */
  template <size_t N>
  libra::Vector<N> TrigonometricInterpolation(const std::vector<double>& time_vector, const std::vector<libra::Vector<N>>& values, double time) const;
  double TrigonometricInterpolation(const std::vector<double>& time_vector, const std::vector<double>& values, double time) const;

  /**
   * @fn LagrangeInterpolation
   * @brief Interpolate with Lagrange method
   * @param [in] time_vector: List of given time
   * @param [in] values: List of given value
   * @param [in] time: Time to calculate the interpolated value
   * @return Interpolated value
   */
  template <size_t N>
  libra::Vector<N> LagrangeInterpolation(const std::vector<double>& time_vector, const std::vector<libra::Vector<N>>& values, double time) const;
  double LagrangeInterpolation(const std::vector<double>& time_vector, const std::vector<double>& values, double time) const;

  std::vector<std::vector<double>> unixtime_vector_;  //!< List of unixtime for all sat
  std::vector<std::vector<double>> time_period_;      //!< List of time period for interpolation
  std::vector<bool> validate_;                        //!< List of whether the satellite is available at the time
  std::vector<int> nearest_index_;                    //!< Index list for update(in position, time_and_index_list_. in clock_bias, time_table_)

  double step_sec_ = 0.0;         //!< Step width [sec]
  double time_interval_ = 0.0;    //!< Time interval
  int interpolation_number_ = 0;  //!< Interpolation number
};

/**
 * @class GnssSat_position
 * @brief Class to manage GNSS satellite position information
 */
class GnssSat_position : public GnssSat_coordinate {
 public:
  /**
   * @fn GnssSat_position
   * @brief Constructor
   */
  GnssSat_position() {}
  /**
   * @fn Init
   * @brief Initialize GNSS satellite position
   * @param[in] file: File path for position calculation
   * @param[in] interpolation_method: Interpolation method for position calculation
   * @param[in] interpolation_number: Interpolation number for position calculation
   * @param[in] ur_flag: Ultra Rapid flag for position calculation
   * @return Start unix time and end unix time
   */
  std::pair<double, double> Init(std::vector<std::vector<std::string>>& file, int interpolation_method, int interpolation_number, UR_KINDS ur_flag);

  /**
   * @fn Setup
   * @brief Setup GNSS satellite position information
   * @param [in] start_unix_time: Start unix time
   * @param [in] step_sec: Step width [sec]
   */
  void SetUp(const double start_unix_time, const double step_sec);
  /**
   * @fn Update
   * @brief Update GNSS satellite position information
   * @param [in] now_unix_time: Current unix time
   */
  void Update(const double now_unix_time);

  /**
   * @fn GetSatEcef
   * @brief Return GNSS satellite position vector in the ECEF frame [m]
   * @param [in] sat_id: GNSS satellite ID defined in this class
   */
  libra::Vector<3> GetSatEcef(int sat_id) const;
  /**
   * @fn GetSatEci
   * @brief Return GNSS satellite position vector in the ECI frame [m]
   * @param [in] sat_id: GNSS satellite ID defined in this class
   */
  libra::Vector<3> GetSatEci(int sat_id) const;

 private:
  std::vector<libra::Vector<3>> gnss_sat_ecef_;  //!< List of GNSS satellite position at specific time in the ECEF frame [m]
  std::vector<libra::Vector<3>> gnss_sat_eci_;   //!< List of GNSS satellite position at specific time in the ECI frame [m]

  std::vector<std::vector<libra::Vector<3>>> gnss_sat_table_ecef_;  //!< Time series of position of all GNSS satellites in the ECEF frame [m]
  std::vector<std::vector<libra::Vector<3>>> gnss_sat_table_eci_;   //!< Time series of position of all GNSS satellites in the ECEF frame [m]

  std::vector<std::vector<libra::Vector<3>>> ecef_;  //!< Time series of position of all GNSS satellites in the ECEF frame before interpolation [m]
  std::vector<std::vector<libra::Vector<3>>> eci_;   //!< Time series of position of all GNSS satellites in the ECEF frame before interpolation [m]
};

/**
 * @class GnssSat_clock
 * @brief Class to manage GNSS satellite clock information
 */
class GnssSat_clock : public GnssSat_coordinate {
 public:
  /**
   * @fn GnssSat_clock
   * @brief Constructor
   */
  GnssSat_clock() {}
  /**
   * @fn Init
   * @brief Initialize GNSS satellite clock
   * @param[in] ile: File path for clock calculation
   * @param[in] file_extension: Extension of the clock file (ex. .sp3, .clk30s)
   * @param[in] interpolation_number: Interpolation number for clock calculation
   * @param[in] ur_flag: Ultra Rapid flag for clock calculation
   */
  void Init(std::vector<std::vector<std::string>>& file, std::string file_extension, int interpolation_number, UR_KINDS ur_flag,
            std::pair<double, double> unix_time_period);
  /**
   * @fn SetUp
   * @brief Setup GNSS satellite clock information
   * @param [in] start_unix_time: Start unix time
   * @param [in] step_sec: Step width [sec]
   */
  void SetUp(const double start_unix_time, const double step_sec);
  /**
   * @fn Update
   * @brief Update GNSS satellite clock information
   * @param [in] now_unix_time: Current unix time
   */
  void Update(const double now_unix_time);
  /**
   * @fn GetSatClock
   * @brief Return GNSS satellite clock in distance expression [m]
   * @param [in] sat_id: GNSS satellite ID defined in this class
   */
  double GetSatClock(int sat_id) const;

 private:
  std::vector<double> gnss_sat_clock_;                     //!< List of clock bias of all GNSS satellites at specific time expressed in distance [m]
  std::vector<std::vector<double>> gnss_sat_clock_table_;  //!< Time series of clock bias of all GNSS satellites expressed in distance [m]
  std::vector<std::vector<double>> clock_bias_;  //!< Time series of clock bias of all GNSS satellites expressed in distance before interpolation [m]
};

/**
 * @class GnssSat_Info
 * @brief Class to manage GNSS satellite information
 */
class GnssSat_Info {
 public:
  /**
   * @fn GnssSat_Info
   * @brief Constructor
   */
  GnssSat_Info();
  /**
   * @fn Init
   * @brief Initialize position and clock
   * @param[in] position_file: File path for position calculation
   * @param[in] position_interpolation_method: Interpolation method for position calculation
   * @param[in] position_interpolation_number: Interpolation number for position calculation
   * @param[in] position_ur_flag: Ultra Rapid flag for position calculation
   * @param[in] clock_file: File path for clock calculation
   * @param[in] clock_file_extension: Extension of the clock file (ex. .sp3, .clk30s)
   * @param[in] clock_interpolation_number: Interpolation number for clock calculation
   * @param[in] clock_ur_flag: Ultra Rapid flag for clock calculation
   */
  void Init(std::vector<std::vector<std::string>>& position_file, int position_interpolation_method, int position_interpolation_number,
            UR_KINDS position_ur_flag, std::vector<std::vector<std::string>>& clock_file, std::string clock_file_extension,
            int clock_interpolation_number, UR_KINDS clock_ur_flag);
  /**
   * @fn SetUp
   * @brief Setup GNSS satellite position and clock information
   * @param [in] start_unix_time: Start unix time
   * @param [in] step_sec: Step width [sec]
   */
  void SetUp(const double start_unix_time, const double step_sec);
  /**
   * @fn Update
   * @brief Update GNSS satellite position and clock information
   * @param [in] now_unix_time: Current unix time
   */
  void Update(const double now_unix_time);

  /**
   * @fn GetNumOfSatellites
   * @brief Get total number of GNSS satellite (constant value)
   * @note TODO: Consider this function is really needed.
   */
  int GetNumOfSatellites() const;
  /**
   * @fn GetWhetherValid
   * @brief Return true the GNSS satellite information is available for both position and clock information
   * @param [in] sat_id: Index of GNSS satellite
   */
  bool GetWhetherValid(int sat_id) const;
  /**
   * @fn GetSatellitePositionEcef
   * @brief Return GNSS satellite position vector in the ECEF frame [m]
   * @param [in] sat_id: GNSS satellite ID defined in this class
   */
  libra::Vector<3> GetSatellitePositionEcef(int sat_id) const;
  /**
   * @fn GetSatellitePositionEci
   * @brief Return GNSS satellite position vector in the ECEF frame [m]
   * @param [in] sat_id: GNSS satellite ID defined in this class
   */
  libra::Vector<3> GetSatellitePositionEci(int sat_id) const;
  /**
   * @fn GetSatelliteClock
   * @brief Return GNSS satellite clock in distance expression [m]
   * @param [in] sat_id: GNSS satellite ID defined in this class
   */
  double GetSatelliteClock(int sat_id) const;
  /**
   * @fn GetGnssSatPos
   * @brief Return GNSS satellite position information class
   */
  const GnssSat_position& GetGnssSatPos() const;
  /**
   * @fn GetGnssSatClock
   * @brief Return GNSS satellite clock information class
   */
  const GnssSat_clock& GetGnssSatClock() const;

 private:
  GnssSat_position position_;  //!< GNSS satellite position information
  GnssSat_clock clock_;        //!< GNSS satellite clock information
};

/**
 * @class GnssSatellites
 * @brief Class to calculate GNSS satellite position and related states
 */
class GnssSatellites : public ILoggable {
 public:
  /**
   * @fn GnssSatellites
   * @brief Constructor
   * @param [in] is_calc_enabled: Flag to manage the GNSS satellite position calculation
   */
  GnssSatellites(bool is_calc_enabled);
  /**
   * @fn ~GnssSatellites
   * @brief Destructor
   */
  virtual ~GnssSatellites() {}

  /**
   * @fn Init
   * @brief Initialize function
   * @note Parameters are defined in GNSSSat_Info for true and estimated information
   */
  void Init(std::vector<std::vector<std::string>>& true_position_file, int true_position_interpolation_method, int true_position_interpolation_number,
            UR_KINDS true_position_ur_flag, std::vector<std::vector<std::string>>& true_clock_file, std::string true_clock_file_extension,
            int true_clock_interpolation_number, UR_KINDS true_clock_ur_flag, std::vector<std::vector<std::string>>& estimate_position_file,
            int estimate_position_interpolation_method, int estimate_position_interpolation_number, UR_KINDS estimate_position_ur_flag,
            std::vector<std::vector<std::string>>& estimate_clock_file, std::string estimate_clock_file_extension,
            int estimate_clock_interpolation_number, UR_KINDS estimate_clock_ur_flag);
  /**
   * @fn IsCalcEnabled
   * @brief Return calculated enabled flag
   */
  bool IsCalcEnabled() const;

  /**
   * @fn SetUp
   * @brief Setup both true and estimated GNSS satellite information
   * @param [in] sim_time: Simulation time information
   */
  void SetUp(const SimTime* sim_time);
  /**
   * @fn Update
   * @brief Update both true and estimated GNSS satellite information
   * @param [in] sim_time: Simulation time information
   */
  void Update(const SimTime* sim_time);

  /**
   * @fn GetIndexFromID
   * @brief Calculate index of GNSS satellite defined in this class from GNSS satellite number defined in GNSS system
   * @note TODO: Is this function really needed? This is just called other accessible function.
   * @return Index of GNSS satellite defined in this class
   */
  int GetIndexFromID(std::string sat_num) const;
  /**
   * @fn GetIDFromIndex
   * @brief Calculate GNSS satellite number defined in GNSS system from index of GNSS satellite defined in this class
   * @note TODO: Is this function really needed? This is just called other accessible function.
   * @return GNSS satellite number defined in GNSS system
   */
  std::string GetIDFromIndex(int index) const;
  /**
   * @fn GetNumOfSatellites
   * @brief Return total number of GNSS satellite for estimated information
   * @note TODO: Is this function really needed? This is just called other accessible function.
   */
  int GetNumOfSatellites() const;
  /**
   * @fn GetWhetherValid
   * @brief Return true the GNSS satellite information is available for both position and clock for both true and estimated value
   * @param [in] sat_id: Index of GNSS satellite
   */
  bool GetWhetherValid(int sat_id) const;
  /**
   * @fn GetStartUnixTime
   * @brief Return start unix time
   */
  double GetStartUnixTime() const;
  /**
   * @fn Get_true_info
   * @brief Return GNSS satellite information class for true value system
   */
  const GnssSat_Info& Get_true_info() const;
  /**
   * @fn Get_estimate_info
   * @brief Return GNSS satellite information class for estimated value system
   */
  const GnssSat_Info& Get_estimate_info() const;

  /**
   * @fn GetSatellitePositionEcef
   * @brief Return GNSS satellite position in the ECEF frame [m]
   * @param [in] sat_id: GNSS satellite ID
   */
  libra::Vector<3> GetSatellitePositionEcef(const int sat_id) const;
  /**
   * @fn GetSatellitePositionEci
   * @brief Return GNSS satellite position in the ECI frame [m]
   * @param [in] sat_id: GNSS satellite ID
   */
  libra::Vector<3> GetSatellitePositionEci(const int sat_id) const;
  /**
   * @fn GetSatelliteClock
   * @brief Return GNSS satellite clock
   * @param [in] sat_id: GNSS satellite ID
   */
  double GetSatelliteClock(const int sat_id) const;

  /**
   * @fn GetPseudoRangeECEF
   * @brief Calculate pseudo range between receiver and a GNSS satellite
   * @param [in] sat_id: GNSS satellite ID
   * @param [in] rec_position: Receiver position vector in the ECEF frame [m]
   * @param [in] rec_clock: Receiver clock
   * @param [in] frequency: Frequency of the signal [MHz]
   * @return Pseudo range [m]
   */
  double GetPseudoRangeECEF(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;
  /**
   * @fn GetPseudoRangeECI
   * @brief Calculate pseudo range between receiver and a GNSS satellite
   * @param [in] sat_id: GNSS satellite ID
   * @param [in] rec_position: Receiver position vector in the ECI frame [m]
   * @param [in] rec_clock: Receiver clock
   * @param [in] frequency: Frequency of the signal [MHz]
   * @return Pseudo range [m]
   */
  double GetPseudoRangeECI(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;
  /**
   * @fn GetCarrierPhaseECEF
   * @brief Calculate carrier phase observed by a receiver for a GNSS satellite
   * @param [in] sat_id: GNSS satellite ID
   * @param [in] rec_position: Receiver position vector in the ECEF frame [m]
   * @param [in] rec_clock: Receiver clock
   * @param [in] frequency: Frequency of the signal [MHz]
   * @return Carrier phase cycle and bias [-]
   */
  std::pair<double, double> GetCarrierPhaseECEF(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;
  /**
   * @fn GetCarrierPhaseECI
   * @brief Calculate carrier phase observed by a receiver for a GNSS satellite
   * @param [in] sat_id: GNSS satellite ID
   * @param [in] rec_position: Receiver position vector in the ECI frame [m]
   * @param [in] rec_clock: Receiver clock
   * @param [in] frequency: Frequency of the signal [MHz]
   * @return Carrier phase cycle and bias [-]
   */
  std::pair<double, double> GetCarrierPhaseECI(const int sat_id, libra::Vector<3> rec_position, double rec_clock, const double frequency) const;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  std::string GetLogHeader() const override;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  std::string GetLogValue() const override;

  /**
   * @fn DebugOutput
   * @brief Debug output
   */
  void DebugOutput(void);

 private:
  /**
   * @fn TrigonometricInterpolation
   * @brief Is this similar with GnssSat_coordinate's function?
   */
  double TrigonometricInterpolation(std::vector<double> time_period, std::vector<double> position, double time);

  /**
   * @fn AddIonosphericDelay
   * @brief Calculation of ionospheric delay
   * @note  TODO: Ionospheric delay very Miscellaneous need to fix
   * @param [in] sat_id: GNSS satellite ID
   * @param [in] rec_position: Receiver position [m]
   * @param [in] frequency: Frequency [MHz]
   * @param [in] flag: The frame definition of the receiver position (ECI or ECEF)
   * @return Ionospheric delay [m]
   */
  double AddIonosphericDelay(const int sat_id, const libra::Vector<3> rec_position, const double frequency, const bool flag) const;

  bool is_calc_enabled_ = true;  //!< Flag to manage the GNSS satellite position calculation
  GnssSat_Info true_info_;       //!< True information of GNSS satellites
  GnssSat_Info estimate_info_;   //!< Estimated information of GNSS satellites TODO: should be move out from GlobalEnvironment
  double start_unix_time_;       //!< Start unix time

#ifdef GNSS_SATELLITES_DEBUG_OUTPUT
  ofstream ofs_true;  //!< Debug output for true value
  ofstream ofs_esti;  //!< Debug output for estimated value
  ofstream ofs_sa;    //!< Debug output for difference between true and estimated value
#endif
};

#endif  // S2E_ENVIRONMENT_GLOBAL_GNSS_SATELLITES_H_
