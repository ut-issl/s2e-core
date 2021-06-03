#pragma once

#include "../../Interface/LogOutput/ILoggable.h"
#include "../Abstract/ComponentBase.h"
#include "../../Library/math/NormalRand.hpp"
#include "../../Dynamics/Dynamics.h"
#include "../../Environment/Global/GnssSatellites.h"
#include "../../Library/math/Quaternion.hpp"
#include "../../Environment/Global/SimTime.h"

#define DEG2RAD 0.017453292519943295769 // PI/180

using libra::Vector;

enum AntennaModel
{
  SIMPLE,
  CONE,
};

typedef struct _gnssinfo
{
  string ID;
  double latitude;
  double longitude;
  double distance;
}GnssInfo;

class GNSSReceiver : public ComponentBase, public ILoggable
{
  public:
    GNSSReceiver(
      const int prescaler,
      ClockGenerator* clock_gen, 
      const int id, 
      const string gnss_id,
      const int ch_max,
      const AntennaModel antenna_model,
      const Vector<3> ant_pos_b,
      const Quaternion q_b2c, 
      const double half_width,
      const Vector<3> noise_std, 
      const Dynamics *dynamics,
      const GnssSatellites *gnss_satellites,
      const SimTime *simtime
    );
    GNSSReceiver(
      const int prescaler,
      ClockGenerator* clock_gen, 
      PowerPort* power_port,
      const int id, 
      string gnss_id,
      const int ch_max,
      const AntennaModel antenna_model,
      const Vector<3> ant_pos_b,
      const Quaternion q_b2c,
      const double half_width,
      const Vector<3> noise_std, 
      const Dynamics *dynamics,
      const GnssSatellites* gnss_satellites,
      const SimTime *simtime
    );
    void MainRoutine(int count);

    // Getter
    inline const GnssInfo GetGnssInfo(int ch) const { return vec_gnssinfo_[ch]; };
    inline const Vector<3> GetPositionECI(void) const { return position_eci_; }
    inline const Vector<3> GetPositionECEF(void) const { return position_ecef_; }
    inline const Vector<3> GetPositionLLH(void) const { return position_llh_; }
    inline const Vector<3> GetVelocityECI(void) const { return velocity_eci_; }
    inline const Vector<3> GetVelocityECEF(void) const { return velocity_ecef_; }

    virtual string GetLogHeader() const;
    virtual string GetLogValue() const;

  protected:
    //Parameters for receiver
    const int id_;  // ID
    const int ch_max_; // Number of channels
    Vector<3> antenna_position_b_;  // GNSS antenna position at body-fixed frame
    Quaternion q_b2c_;  // Quaternion from body frame to component frame
    libra::NormalRand nrs_eci_x_, nrs_eci_y_, nrs_eci_z_; // Random Error for each axis
    double half_width_ = 0.0;
    string gnss_id_;
    AntennaModel antenna_model_;

    //Calculated values
    Vector<3> position_eci_{ 0.0 };
    Vector<3> velocity_eci_{ 0.0 };     // [m]
    Vector<3> position_ecef_{ 0.0 };    // [m/s]
    Vector<3> velocity_ecef_{ 0.0 };    // [m/s]
    Vector<3> position_llh_{ 0.0 };     // [rad,rad,m]
    UTC utc_ = {2000, 1, 1, 0, 0, 0.0}; // [year, month, day, hour, min, sec]
    unsigned int gpstime_week_ = 0;
    double       gpstime_sec_  = 0.0;
    int is_gnss_sats_visible_  = 0;
    int gnss_sats_visible_num_ = 0;
    std::vector<GnssInfo> vec_gnssinfo_;
    
    //References
    const Dynamics* dynamics_;
    const GnssSatellites* gnss_satellites_;
    const SimTime*  simtime_;

    //Internal Functions
    void CheckAntenna(Vector<3> location_true, Quaternion q_i2b);
    void CheckAntennaSimple(Vector<3> location_true, Quaternion q_i2b);
    void CheckAntennaCone(Vector<3> location_true, Quaternion q_i2b);
    void SetGnssInfo(Vector<3> ant2gnss_i, Quaternion q_i2b, string gnss_id);
    void AddNoise(Vector<3> location_true_eci, Vector<3> location_true_ecef); // substitutional method for "Measure" in other sensor modles inherited SensorBase class
    void ConvertJulianDayToGPSTime(const double JulianDay);
};



