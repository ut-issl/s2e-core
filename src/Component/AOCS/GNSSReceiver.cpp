#include <string>
#include "GNSSReceiver.h"
#include "../../Library/math/GlobalRand.h"


GNSSReceiver::GNSSReceiver(
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
  const SimTime *simtime)
  : ComponentBase(prescaler, clock_gen), id_(id), gnss_id_(gnss_id), ch_max_(ch_max), antenna_model_(antenna_model), antenna_position_b_(ant_pos_b), q_b2c_(q_b2c), half_width_(half_width),
    nrs_eci_x_(0.0, noise_std[0], g_rand.MakeSeed()), nrs_eci_y_(0.0, noise_std[1], g_rand.MakeSeed()), nrs_eci_z_(0.0, noise_std[2], g_rand.MakeSeed()),
    dynamics_(dynamics), gnss_satellites_(gnss_satellites),simtime_(simtime)
{}
GNSSReceiver::GNSSReceiver(
  const int prescaler,
  ClockGenerator* clock_gen,
  PowerPort* power_port,
  const int id,
  const string gnss_id,
  const int ch_max,
  const AntennaModel antenna_model,
  const Vector<3> ant_pos_b,
  const Quaternion q_b2c,
  const double half_width,
  const Vector<3> noise_std,
  const Dynamics* dynamics,
  const GnssSatellites* gnss_satellites,
  const SimTime *simtime)
  : ComponentBase(prescaler, clock_gen, power_port), id_(id), gnss_id_(gnss_id), ch_max_(ch_max), antenna_model_(antenna_model), antenna_position_b_(ant_pos_b), q_b2c_(q_b2c), half_width_(half_width),
    nrs_eci_x_(0.0, noise_std[0], g_rand.MakeSeed()), nrs_eci_y_(0.0, noise_std[1], g_rand.MakeSeed()), nrs_eci_z_(0.0, noise_std[2], g_rand.MakeSeed()),
    dynamics_(dynamics), gnss_satellites_(gnss_satellites), simtime_(simtime)
{}

void GNSSReceiver::MainRoutine(int count)
{
  Vector<3> pos_true_eci_ = dynamics_->GetOrbit().GetSatPosition_i();
  Quaternion q_i2b = dynamics_->GetQuaternion_i2b();
  
  CheckAntenna(pos_true_eci_, q_i2b);
  
  if (is_gnss_sats_visible_ == 1) {  //Antenna of GNSS-R can detect GNSS signal
    position_ecef_  = dynamics_->GetOrbit().GetSatPosition_ecef();
    position_llh_   = dynamics_->GetOrbit().GetLatLonAlt();
    velocity_ecef_  = dynamics_->GetOrbit().GetSatVelocity_ecef();
    AddNoise(pos_true_eci_, position_ecef_);

    utc_            = simtime_->GetCurrentUTC();
    ConvertJulianDayToGPSTime(simtime_->GetCurrentJd());
  }
  else{
    // position information will not be updated in this case
    // only time information will be updated in this case (according to the receiver's internal clock)
    utc_ = simtime_->GetCurrentUTC();
    ConvertJulianDayToGPSTime(simtime_->GetCurrentJd());
  }
}

void GNSSReceiver::CheckAntenna(const Vector<3> pos_true_eci_, Quaternion q_i2b)
{
  if (antenna_model_ == SIMPLE)
    CheckAntennaSimple(pos_true_eci_, q_i2b);
  else if (antenna_model_ == CONE)
    CheckAntennaCone(pos_true_eci_, q_i2b);
}

void GNSSReceiver::CheckAntennaSimple(const Vector<3> pos_true_eci_, Quaternion q_i2b)
{
  //Simplest model
  //GNSS sats are visible when antenna directs anti-earth direction
  // antenna normal vector at inertial frame
  Vector<3> antenna_direction_c(0.0);
  antenna_direction_c[2] = 1.0;
  Vector<3> antenna_direction_b = q_b2c_.frame_conv_inv(antenna_direction_c);
  Vector<3> antenna_direction_i = q_i2b.frame_conv_inv(antenna_direction_b);

  double inner = inner_product(pos_true_eci_, antenna_direction_i);
  if (inner <= 0) 
    is_gnss_sats_visible_ = 0;
  else 
    is_gnss_sats_visible_ = 1;
}

void GNSSReceiver::CheckAntennaCone(const Vector<3> pos_true_eci_, Quaternion q_i2b)
{
  // Cone model
  Vector<3> gnss_sat_pos_i, ant_pos_i, ant2gnss_i, ant2gnss_i_n, sat2ant_i;
  vec_gnssinfo_.clear();
  
  // antenna normal vector at inertial frame
  Vector<3> antenna_direction_c(0.0);
  antenna_direction_c[2] = 1.0;
  Vector<3> antenna_direction_b = q_b2c_.frame_conv_inv(antenna_direction_c);
  Vector<3> antenna_direction_i = q_i2b.frame_conv_inv(antenna_direction_b);
  
  sat2ant_i = q_i2b.frame_conv_inv(antenna_position_b_);
  ant_pos_i = pos_true_eci_ + sat2ant_i;

  // initialize
  gnss_sats_visible_num_ = 0;

  int gnss_num = gnss_satellites_->GetNumOfSatellites();

  for (int i = 0; i < gnss_num; i++)
  {
    // check if gnss ID is compatible with the receiver
    string id_tmp = gnss_satellites_->GetIDFromIndex(i);
    if (gnss_id_.find(id_tmp[0]) == string::npos) continue;
    
    // compute direction from sat to gnss in body-fixed frame
    gnss_sat_pos_i = gnss_satellites_->GetSatellitePositionEci(i);
    ant2gnss_i = gnss_sat_pos_i - ant_pos_i;
    double normalizer = 1/norm(ant2gnss_i);
    ant2gnss_i_n = normalizer*ant2gnss_i;

    // check gnss sats are visible from antenna
    double Re = 6378137.0; // radius of earth [m] 
    double inner1 = inner_product(ant_pos_i, gnss_sat_pos_i);
    int is_visible_ant2gnss = 0;
    if (inner1 > 0)
      is_visible_ant2gnss = 1;
    else
    {
      Vector<3> tmp = ant_pos_i + inner_product(-ant_pos_i, ant2gnss_i_n) * ant2gnss_i;
      if (norm(tmp) < Re)
        // There is earth between antenna and gnss
        is_visible_ant2gnss = 0;
      else
        // There is not earth between antenna and gnss
        is_visible_ant2gnss = 1;
    }

    double inner2 = inner_product(antenna_direction_i, ant2gnss_i_n);
    if (inner2 > cos(half_width_*DEG2RAD) && is_visible_ant2gnss)
    {
      // is visible
      gnss_sats_visible_num_++;
      SetGnssInfo(ant2gnss_i, q_i2b, id_tmp);
    }
  }

  if (gnss_sats_visible_num_ > 0)
    is_gnss_sats_visible_ = 1;
  else
    is_gnss_sats_visible_ = 0;
}

void GNSSReceiver::SetGnssInfo(Vector<3> ant2gnss_i, Quaternion q_i2b, string gnss_id)
{
  Vector<3> ant2gnss_b, ant2gnss_c;

  ant2gnss_b = q_i2b.frame_conv(ant2gnss_i);
  ant2gnss_c = q_b2c_.frame_conv(ant2gnss_b);

  double dist = norm(ant2gnss_c);
  double lon = AcTan(ant2gnss_c[1], ant2gnss_c[0]);
  double lat = AcTan(ant2gnss_c[2], dist);

  GnssInfo gnss_info_new = { gnss_id, lat, lon, dist };
  vec_gnssinfo_.push_back(gnss_info_new);
}

void GNSSReceiver::AddNoise(Vector<3> location_true_eci, Vector<3> location_true_ecef)
{
  //Simplest noise model
  position_eci_[0] = location_true_eci[0] + nrs_eci_x_;
  position_eci_[1] = location_true_eci[1] + nrs_eci_y_;
  position_eci_[2] = location_true_eci[2] + nrs_eci_z_;

  position_ecef_[0] = location_true_ecef[0] + nrs_eci_x_;
  position_ecef_[1] = location_true_ecef[1] + nrs_eci_y_;
  position_ecef_[2] = location_true_ecef[2] + nrs_eci_z_;
}

void GNSSReceiver::ConvertJulianDayToGPSTime(const double JulianDay)
{
  const double kJulianDayAtGPSTimeZero = 2444244.5; // corresponds to 1980/1/5 midnight
  const double kDayInWeek = 7.0;
  const double kSecInWeek = 604800.0;
  const double kSecInDay  = 86400.0;

  // compute ToW from current JulianDay 
  // note:"gpstime_week_" computed in this method is larger than 1024
  double elapsed_day = JulianDay - kJulianDayAtGPSTimeZero;
  gpstime_week_ = (unsigned int)(elapsed_day / kDayInWeek);
  gpstime_sec_ = (elapsed_day - (double)(gpstime_week_)*kDayInWeek) * kSecInDay;
}

string GNSSReceiver::GetLogHeader() const //For logs
{
  string str_tmp = "";
  str_tmp += WriteScalar("gnss_year");
  str_tmp += WriteScalar("gnss_month");
  str_tmp += WriteScalar("gnss_day");
  str_tmp += WriteScalar("gnss_hour");
  str_tmp += WriteScalar("gnss_min");
  str_tmp += WriteScalar("gnss_sec");
  str_tmp += WriteVector("gnss_position", "eci", "m", 3);
  str_tmp += WriteVector("gnss_velocity", "ecef", "m/s", 3);
  str_tmp += WriteScalar("gnss_lat","rad");
  str_tmp += WriteScalar("gnss_lon","rad");
  str_tmp += WriteScalar("gnss_alt","m");
  str_tmp += WriteScalar("gnss_vis_flag");
  str_tmp += WriteScalar("gnss_vis_num");

  return str_tmp;
}

string GNSSReceiver::GetLogValue() const //For logs
{
  string str_tmp = "";
  str_tmp += WriteScalar(utc_.year);
  str_tmp += WriteScalar(utc_.month);
  str_tmp += WriteScalar(utc_.day);
  str_tmp += WriteScalar(utc_.hour);
  str_tmp += WriteScalar(utc_.min);
  str_tmp += WriteScalar(utc_.sec);
  str_tmp += WriteVector(position_eci_,10);
  str_tmp += WriteVector(velocity_ecef_,10);
  str_tmp += WriteScalar(position_llh_[0],10);
  str_tmp += WriteScalar(position_llh_[1],10);
  str_tmp += WriteScalar(position_llh_[2],10);
  str_tmp += WriteScalar(is_gnss_sats_visible_);
  str_tmp += WriteScalar(gnss_sats_visible_num_);

  return str_tmp;
}
