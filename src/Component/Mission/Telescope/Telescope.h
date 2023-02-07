/*
 * @file Telescope.h
 * @brief Component emulation: Telescope
 */

#pragma once
#include <Abstract/ComponentBase.h>
#include <Interface/LogOutput/ILoggable.h>

#include <Library/math/Quaternion.hpp>
#include <Library/math/Vector.hpp>
#include <dynamics/attitude/attitude.hpp>
#include <environment/global/hipparcos_catalogue.hpp>
#include <environment/local/local_celestial_information.hpp>
#include <vector>

/*
 * @struct Star
 * @brief Information of stars in the telescope's field of view
 */
struct Star {
  HipData hipdata;                 //!< Hipparcos data
  libra::Vector<2> pos_imgsensor;  //!< Position of image sensor
};

/*
 * @class Telescope
 * @brief Component emulation: Telescope
 */
class Telescope : public ComponentBase, public ILoggable {
 public:
  Telescope(ClockGenerator* clock_gen, libra::Quaternion& q_b2c, double sun_forbidden_angle, double earth_forbidden_angle,
            double moon_forbidden_angle, int x_num_of_pix, int y_num_of_pix, double x_fov_par_pix, double y_fov_par_pix, size_t num_of_logged_stars,
            const Attitude* attitude, const HipparcosCatalogue* hipp, const LocalCelestialInformation* local_celes_info);

  ~Telescope();

  // Getter
  inline bool GetIsSunInForbiddenAngle() const { return is_sun_in_forbidden_angle; }
  inline bool GetIsEarthInForbiddenAngle() const { return is_earth_in_forbidden_angle; }
  inline bool GetIsMoonInForbiddenAngle() const { return is_moon_in_forbidden_angle; }

 protected:
 private:
  libra::Quaternion q_b2c_;  //!< Quaternion from the body frame to component frame
  libra::Vector<3> sight_;   //!< Sight direction vector in the component frame

  double sun_forbidden_angle_;    //!< Sun forbidden angle [rad]
  double earth_forbidden_angle_;  //!< Earth forbidden angle [rad]
  double moon_forbidden_angle_;   //!< Moon forbidden angle [rad]

  int x_num_of_pix_;           //!< Number of pixel on X-axis in the image plane
  int y_num_of_pix_;           //!< Number of pixel on Y-axis in the image plane
  double x_fov_par_pix_;       //!< Field of view per pixel of X-axis in the image plane [rad/pix]
  double y_fov_par_pix_;       //!< Field of view per pixel of Y-axis in the image plane [rad/pix]
  double x_field_of_view_rad;  //!< Field of view of X-axis in the image plane [rad/pix]
  double y_field_of_view_rad;  //!< Field of view of Y-axis in the image plane [rad/pix]

  bool is_sun_in_forbidden_angle = false;    //!< Is the sun in the forbidden angle
  bool is_earth_in_forbidden_angle = false;  //!< Is the earth in the forbidden angle
  bool is_moon_in_forbidden_angle = false;   //!< Is the moon in the forbidden angle

  size_t num_of_logged_stars_;  //!< Number of logged stars

  libra::Vector<2> sun_pos_imgsensor{-1};    //!< Position of the sun on the image plane
  libra::Vector<2> earth_pos_imgsensor{-1};  //!< Position of the earth on the image plane
  libra::Vector<2> moon_pos_imgsensor{-1};   //!< Position of the moon on the image plane

  std::vector<Star> star_in_sight;  //!< Star information in the field of view

  /**
   * @fn JudgeForbiddenAngle
   * @brief Judge the forbidden angles are violated
   * @param [in] target_b: Direction vector of target on the body fixed frame
   * @param [in] forbidden_angle: Forbidden angle [rad]
   */
  bool JudgeForbiddenAngle(const libra::Vector<3>& target_b, const double forbidden_angle);

  // Override functions for ComponentBase
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
   */
  void MainRoutine(int count);

  /**
   * @fn Observe
   * @brief Convert body fixed direction vector to position on image sensor plane
   * @param [out] pos_imgsensor: Position on image sensor plane
   * @param [in] target_b: Direction vector of target on the body fixed frame
   */
  void Observe(Vector<2>& pos_imgsensor, const Vector<3, double> target_b);
  /**
   * @fn ObserveStars
   * @brief Observe stars from Hipparcos catalogue
   */
  void ObserveStars();

  const Attitude* attitude_;                           //!< Attitude information
  const HipparcosCatalogue* hipp_;                     //!< Star information
  const LocalCelestialInformation* local_celes_info_;  //!< Local celestial information

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

  // For debug **********************************************
  //  Vector<3> sun_pos_c;
  //  Vector<3> earth_pos_c;
  //  Vector<3> moon_pos_c;
  //  double angle_sun;
  //  double angle_earth;
  //  double angle_moon;
  //*************************************************************
};
