#pragma once
#include <Abstract/ComponentBase.h>
#include <Dynamics/Attitude/Attitude.h>
#include <Environment/Global/HipparcosCatalogue.h>
#include <Environment/Local/LocalCelestialInformation.h>
#include <Interface/LogOutput/ILoggable.h>

#include <Library/math/Quaternion.hpp>
#include <Library/math/Vector.hpp>
#include <vector>

struct Star  //望遠鏡視野内に入っている恒星の情報
{
  HipData hipdata;
  libra::Vector<2> pos_imgsensor;
};

class Telescope : public ComponentBase, public ILoggable {
 public:
  Telescope(ClockGenerator* clock_gen, libra::Quaternion& q_b2c, double sun_forbidden_angle, double earth_forbidden_angle,
            double moon_forbidden_angle, int x_num_of_pix, int y_num_of_pix, double x_fov_par_pix, double y_fov_par_pix, size_t num_of_logged_stars,
            const Attitude* attitude, const HipparcosCatalogue* hipp, const LocalCelestialInformation* local_celes_info);

  ~Telescope();

  // Getter
  inline bool GetIsSunInForbiddenAngle() const {return is_sun_in_forbidden_angle;}
  inline bool GetIsEarthInForbiddenAngle() const {return is_earth_in_forbidden_angle;}
  inline bool GetIsMoonInForbiddenAngle() const {return is_moon_in_forbidden_angle;}

 protected:
 private:
  //! 望遠鏡の姿勢b2c
  libra::Quaternion q_b2c_;
  //! 視線方向ベクトル(コンポ座標系表記)
  libra::Vector<3> sight_;

  double sun_forbidden_angle_;
  double earth_forbidden_angle_;
  double moon_forbidden_angle_;

  int x_num_of_pix_;
  int y_num_of_pix_;
  double x_fov_par_pix_;  //単位：rad
  double y_fov_par_pix_;  //単位：rad
  double x_field_of_view_rad;
  double y_field_of_view_rad;

  bool is_sun_in_forbidden_angle = false;
  bool is_earth_in_forbidden_angle = false;
  bool is_moon_in_forbidden_angle = false;

  size_t num_of_logged_stars_;  //恒星観測でログに出力する恒星の個数

  libra::Vector<2> sun_pos_imgsensor{-1};    //イメージセンサ上における太陽の像の位置
  libra::Vector<2> earth_pos_imgsensor{-1};  //イメージセンサ上における地球の像の位置
  libra::Vector<2> moon_pos_imgsensor{-1};   //イメージセンサ上における月の像の位置

  std::vector<Star> star_in_sight;

  bool JudgeForbiddenAngle(const libra::Vector<3>& target_b, const double forbidden_angle);
  void MainRoutine(int count);
  void Observe(Vector<2>& pos_imgsensor, const Vector<3, double> target_b);
  void ObserveStars();  // Hip Catalogueのデータからの観測

  const Attitude* attitude_;
  const HipparcosCatalogue* hipp_;
  const LocalCelestialInformation* local_celes_info_;

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  //デバッグ用変数**********************************************
  // Vector<3> sun_pos_c;
  // Vector<3> earth_pos_c;
  // Vector<3> moon_pos_c;
  // double angle_sun;
  // double angle_earth;
  // double angle_moon;
  //*************************************************************
};
