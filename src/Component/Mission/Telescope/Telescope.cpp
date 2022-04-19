#include "Telescope.h"

#include <Library/math/Constant.hpp>
#include <cassert>

using namespace std;
using namespace libra;

Telescope::Telescope(ClockGenerator* clock_gen, libra::Quaternion& q_b2c, double sun_forbidden_angle, double earth_forbidden_angle,
                     double moon_forbidden_angle, int x_num_of_pix, int y_num_of_pix, double x_fov_par_pix, double y_fov_par_pix,
                     int num_of_logged_stars, const Attitude* attitude, const HipparcosCatalogue* hipp,
                     const LocalCelestialInformation* local_celes_info)
    : ComponentBase(1, clock_gen),
      q_b2c_(q_b2c),
      sun_forbidden_angle_(sun_forbidden_angle),
      earth_forbidden_angle_(earth_forbidden_angle),
      moon_forbidden_angle_(moon_forbidden_angle),
      x_num_of_pix_(x_num_of_pix),
      y_num_of_pix_(y_num_of_pix),
      x_fov_par_pix_(x_fov_par_pix),
      y_fov_par_pix_(y_fov_par_pix),
      num_of_logged_stars_(num_of_logged_stars),
      attitude_(attitude),
      hipp_(hipp),
      local_celes_info_(local_celes_info) {
  is_sun_in_forbidden_angle = true;
  is_earth_in_forbidden_angle = true;
  is_moon_in_forbidden_angle = true;

  x_field_of_view_rad = x_num_of_pix_ * x_fov_par_pix_;
  y_field_of_view_rad = y_num_of_pix_ * y_fov_par_pix_;
  assert(x_field_of_view_rad < libra::pi_2);  //視野角90度以上だと計算が成り立たないので，その場合を弾く
  assert(y_field_of_view_rad < libra::pi_2);

  sight_ = Vector<3>(0);
  sight_[0] = 1;  //(1,0,0)@コンポ座標，視線方向ベクトル

  // t=0のときの値として0を入れておく
  for (unsigned int i = 0; i < num_of_logged_stars_; i++) {
    Star star;
    star.hipdata.hip_num = -1;
    star.hipdata.vmag = -1;
    star.hipdata.ra = -1;
    star.hipdata.de = -1;
    star.pos_imgsensor[0] = -1;
    star.pos_imgsensor[1] = -1;

    star_in_sight.push_back(star);
  }
}

Telescope::~Telescope() {}

void Telescope::MainRoutine(int count) {
  //禁止角判定
  is_sun_in_forbidden_angle = JudgeForbiddenAngle(local_celes_info_->GetPosFromSC_b("SUN"), sun_forbidden_angle_);
  is_earth_in_forbidden_angle = JudgeForbiddenAngle(local_celes_info_->GetPosFromSC_b("EARTH"), earth_forbidden_angle_);
  is_moon_in_forbidden_angle = JudgeForbiddenAngle(local_celes_info_->GetPosFromSC_b("MOON"), moon_forbidden_angle_);
  // CelesInfoから得られる各天体の像の位置の計算
  Observe(sun_pos_imgsensor, local_celes_info_->GetPosFromSC_b("SUN"));
  Observe(earth_pos_imgsensor, local_celes_info_->GetPosFromSC_b("EARTH"));
  Observe(moon_pos_imgsensor, local_celes_info_->GetPosFromSC_b("MOON"));
  // HIP Catalogueから得られる、画角内の天体のHIP IDとセンサ上の位置の計算
  // HIP Catalogueがデータを読まなかった場合は，アップデートしない
  if (hipp_->IsCalcEnabled) ObserveStars();
  //以下デバッグ******************************************************************
  // sun_pos_c =
  // q_b2c_.frame_conv(dynamics_->celestial_->GetPosFromSC_b("SUN"));
  // earth_pos_c =
  // q_b2c_.frame_conv(dynamics_->celestial_->GetPosFromSC_b("EARTH"));
  // moon_pos_c =
  // q_b2c_.frame_conv(dynamics_->celestial_->GetPosFromSC_b("MOON")); angle_sun
  // = angle(sight_, sun_pos_c) * 180/libra::pi; angle_earth = angle(sight_,
  // earth_pos_c) * 180 / libra::pi; angle_moon = angle(sight_, moon_pos_c) *
  // 180 / libra::pi;
  //******************************************************************************
}

bool Telescope::JudgeForbiddenAngle(const libra::Vector<3>& target_b, const double forbidden_angle) {
  Quaternion q_c2b = q_b2c_.conjugate();
  Vector<3> sight_b = q_c2b.frame_conv(sight_);
  double angle_rad = libra::angle(target_b, sight_b);
  if (angle_rad < forbidden_angle) {
    return true;
  } else
    return false;
}

void Telescope::Observe(Vector<2>& pos_imgsensor, const Vector<3, double> target_b) {
  Vector<3, double> target_c = q_b2c_.frame_conv(target_b);
  double arg_x = atan2(target_c[2], target_c[0]);  //コンポ座標xz平面上でx軸から測った偏角
  double arg_y = atan2(target_c[1], target_c[0]);  //コンポ座標xy平面上でx軸から測った偏角

  if (abs(arg_x) < x_field_of_view_rad && abs(arg_y) < y_field_of_view_rad) {
    pos_imgsensor[0] = x_num_of_pix_ / 2 * tan(arg_x) / tan(x_field_of_view_rad) + x_num_of_pix_ / 2;
    pos_imgsensor[1] = y_num_of_pix_ / 2 * tan(arg_y) / tan(y_field_of_view_rad) + y_num_of_pix_ / 2;
  } else {  //天体が画角外の場合は-1を出力
    pos_imgsensor[0] = -1;
    pos_imgsensor[1] = -1;
  }
}

void Telescope::ObserveStars() {
  Quaternion q_i2b = attitude_->GetQuaternion_i2b();

  star_in_sight.clear();  //最初にクリアしておく
  int count = 0;          // whileループのカウンタ

  while (star_in_sight.size() < num_of_logged_stars_) {
    Vector<3> target_b = hipp_->GetStarDir_b(count, q_i2b);
    Vector<3> target_c = q_b2c_.frame_conv(target_b);

    double arg_x = atan2(target_c[2],
                         target_c[0]);  //コンポ座標xz平面上でx軸から測った偏角
    double arg_y = atan2(target_c[1],
                         target_c[0]);  //コンポ座標xy平面上でx軸から測った偏角

    if (abs(arg_x) <= x_field_of_view_rad && abs(arg_y) <= y_field_of_view_rad) {
      Star star;
      star.hipdata.hip_num = hipp_->GetHipID(count);
      star.hipdata.vmag = hipp_->GetVmag(count);
      star.hipdata.ra = hipp_->GetRA(count);
      star.hipdata.de = hipp_->GetDE(count);
      star.pos_imgsensor[0] = x_num_of_pix_ / 2.0 * tan(arg_x) / tan(x_field_of_view_rad) + x_num_of_pix_ / 2.0;
      star.pos_imgsensor[1] = y_num_of_pix_ / 2.0 * tan(arg_y) / tan(y_field_of_view_rad) + y_num_of_pix_ / 2.0;

      star_in_sight.push_back(star);
    }

    count++;

    //カタログをすべて見尽くしてしまった場合に，残りの各種パラメータを-1で埋めてループから抜ける
    if (count >= hipp_->GetCatalogueSize()) {
      while (star_in_sight.size() < num_of_logged_stars_) {
        Star star;
        star.hipdata.hip_num = -1;
        star.hipdata.vmag = -1;
        star.hipdata.ra = -1;
        star.hipdata.de = -1;
        star.pos_imgsensor[0] = -1;
        star.pos_imgsensor[1] = -1;

        star_in_sight.push_back(star);
      }

      break;
    }
  }
}

string Telescope::GetLogHeader() const {
  string str_tmp = "";
  str_tmp += WriteScalar("Sun in forbidden angle", "");
  str_tmp += WriteScalar("Earth in forbidden angle", "");
  str_tmp += WriteScalar("Moon in forbidden angle", "");
  str_tmp += WriteVector("sun_pos_imgsensor", " ", "pix", 2);
  str_tmp += WriteVector("earth_pos_imgsensor", " ", "pix", 2);
  str_tmp += WriteVector("moon_pos_imgsensor", " ", "pix", 2);
  // Hip
  // Catalogueがデータを読まなかった場合は，ObserveStarsに関する出力を行わない
  if (hipp_->IsCalcEnabled) {
    for (unsigned int i = 0; i < num_of_logged_stars_; i++) {
      str_tmp += WriteScalar("HIP ID (" + to_string(i) + ")", " ");
      str_tmp += WriteScalar("Vmag (" + to_string(i) + ")", " ");
      str_tmp += WriteVector("pos_imagesensor (" + to_string(i) + ")", " ", "pix", 2);
    }
  }

  //デバッグ出力**********************************************
  // str_tmp += WriteScalar("angle_sun", "");
  // str_tmp += WriteScalar("angle_earth", "");
  // str_tmp += WriteScalar("angle_moon", "");
  //**********************************************************
  return str_tmp;
}

string Telescope::GetLogValue() const {
  string str_tmp = "";
  str_tmp += WriteScalar(is_sun_in_forbidden_angle);
  str_tmp += WriteScalar(is_earth_in_forbidden_angle);
  str_tmp += WriteScalar(is_moon_in_forbidden_angle);
  str_tmp += WriteVector(sun_pos_imgsensor);
  str_tmp += WriteVector(earth_pos_imgsensor);
  str_tmp += WriteVector(moon_pos_imgsensor);
  // Hip
  // Catalogueがデータを読まなかった場合は，ObserveStarsに関する出力を行わない
  if (hipp_->IsCalcEnabled) {
    for (unsigned int i = 0; i < num_of_logged_stars_; i++) {
      str_tmp += WriteScalar(star_in_sight[i].hipdata.hip_num);
      str_tmp += WriteScalar(star_in_sight[i].hipdata.vmag);
      str_tmp += WriteVector(star_in_sight[i].pos_imgsensor);
    }
  }

  //デバッグ出力**********************************************
  // str_tmp += WriteScalar(angle_sun);
  // str_tmp += WriteScalar(angle_earth);
  // str_tmp += WriteScalar(angle_moon);
  //**********************************************************
  return str_tmp;
}
