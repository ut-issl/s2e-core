#include <string>
#include "GNSSReceiver.h"
#include "../../Library/math/GlobalRand.h"


GNSSReceiver::GNSSReceiver(const int id, const Vector<3> ant_direction, const Vector<3> noise_std, const Dynamics *dynamics)
	: ComponentBase(50), id_(id), antenna_direction_(ant_direction),
    nrs_eci_x_(0.0, noise_std[0], g_rand.MakeSeed()), nrs_eci_y_(0.0, noise_std[1], g_rand.MakeSeed()), nrs_eci_z_(0.0, noise_std[2], g_rand.MakeSeed()),
    dynamics_(dynamics)
{
  position_eci_[0] = 0.0;
  position_eci_[1] = 0.0;
  position_eci_[2] = 0.0;
  is_gnss_sats_visible_ = 0;
}

void GNSSReceiver::MainRoutine(int count)
{
  Vector<3> pos_ture_eci_ = dynamics_->GetOrbit().GetSatPosition_i();
  CheckAntenna(pos_ture_eci_);
  if (is_gnss_sats_visible_ == 1) {  //Antenna of GNSS-R can detect GNSS signal
    AddNoise(pos_ture_eci_);
  }
  else{
    position_eci_[0] = 0.0;
    position_eci_[1] = 0.0;
    position_eci_[2] = 0.0;
  }
}

void GNSSReceiver::CheckAntenna(const Vector<3> pos_true_eci_)
{
  //Simplest model
  //GNSS sats are visible when antenna directs anti-earth direction
  double inner = inner_product(pos_true_eci_, antenna_direction_);
  if (inner <= 0) {	
    is_gnss_sats_visible_ = 0;
  }
  else {
    is_gnss_sats_visible_ = 1;
  }
  return;
}

void GNSSReceiver::AddNoise(Vector<3> pos_true_eci_)
{
  //Simplest noise model
  position_eci_[0] = pos_true_eci_[0] + nrs_eci_x_;
  position_eci_[1] = pos_true_eci_[1] + nrs_eci_y_;
  position_eci_[2] = pos_true_eci_[2] + nrs_eci_z_;
}

string GNSSReceiver::GetLogHeader() const //For logs
{
  string str_tmp = "";
  str_tmp += WriteVector("gnss_position", "eci", "m", 3);
  str_tmp += WriteScalar("gnss_vis_flag");

  return str_tmp;
}

string GNSSReceiver::GetLogValue() const //For logs
{
  string str_tmp = "";
  str_tmp += WriteVector(position_eci_);
  str_tmp += WriteScalar(is_gnss_sats_visible_);

  return str_tmp;
}
