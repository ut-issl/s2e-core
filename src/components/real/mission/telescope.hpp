/*
 * @file telescope.hpp
 * @brief Component emulation: Telescope
 */

#ifndef S2E_COMPONENTS_REAL_MISSION_TELESCOPE_HPP_P_
#define S2E_COMPONENTS_REAL_MISSION_TELESCOPE_HPP_P_

#include <dynamics/attitude/attitude.hpp>
#include <dynamics/orbit/orbit.hpp>
#include <environment/global/hipparcos_catalogue.hpp>
#include <environment/local/local_celestial_information.hpp>
#include <library/logger/loggable.hpp>
#include <library/math/quaternion.hpp>
#include <library/math/vector.hpp>
#include <vector>

#include "../../base/component.hpp"

/*
 * @struct Star
 * @brief Information of stars in the telescope's field of view
 */
struct Star {
  HipparcosData hipparcos_data;            //!< Hipparcos data
  libra::Vector<2> position_image_sensor;  //!< Position of image sensor
};

/*
 * @class Telescope
 * @brief Component emulation: Telescope
 */
class Telescope : public Component, public ILoggable {
 public:
  /**
   * @fn Telescope
   * @brief Constructor
   * @param [in] clock_generator: Clock Generator
   * @param [in] quaternion_b2c: Frame conversion Quaternion from body to component frame
   * @param [in] sun_forbidden_angle_rad: Sun forbidden angle [rad]
   * @param [in] earth_forbidden_angle_rad: Earth forbidden angle [rad]
   * @param [in] moon_forbidden_angle_rad: Moon forbidden angle [rad]
   * @param [in] x_number_of_pix: Number of pixel on X-axis in the image plane
   * @param [in] y_number_of_pix: Number of pixel on Y-axis in the image plane
   * @param [in] x_fov_per_pix: Field of view per pixel of X-axis in the image plane [rad/pix]
   * @param [in] y_fov_per_pix: Field of view per pixel of Y-axis in the image plane [rad/pix]
   * @param [in] number_of_logged_stars: Number of logged stars
   * @param [in] attitude: Attitude Information
   * @param [in] hipparcos: Hipparcos catalogue information
   * @param [in] local_celestial_information: Local celestial information
   * @param [in] orbit: Orbit information
   */
  Telescope(ClockGenerator* clock_generator, const libra::Quaternion& quaternion_b2c, const double sun_forbidden_angle_rad,
            const double earth_forbidden_angle_rad, const double moon_forbidden_angle_rad, const int x_number_of_pix, const int y_number_of_pix,
            const double x_fov_per_pix, const double y_fov_per_pix, size_t number_of_logged_stars, const Attitude* attitude,
            const HipparcosCatalogue* hipparcos, const LocalCelestialInformation* local_celestial_information, const Orbit* orbit = nullptr);
  /**
   * @fn ~Telescope
   * @brief Destructor
   */
  ~Telescope();

  // Getter
  inline bool GetIsSunInForbiddenAngle() const { return is_sun_in_forbidden_angle; }
  inline bool GetIsEarthInForbiddenAngle() const { return is_earth_in_forbidden_angle; }
  inline bool GetIsMoonInForbiddenAngle() const { return is_moon_in_forbidden_angle; }

 protected:
 private:
  libra::Quaternion quaternion_b2c_;    //!< Quaternion from the body frame to component frame
  libra::Vector<3> sight_direction_c_;  //!< Sight direction vector in the component frame

  double sun_forbidden_angle_rad_;    //!< Sun forbidden angle [rad]
  double earth_forbidden_angle_rad_;  //!< Earth forbidden angle [rad]
  double moon_forbidden_angle_rad_;   //!< Moon forbidden angle [rad]

  int x_number_of_pix_;                          //!< Number of pixel on X-axis in the image plane
  int y_number_of_pix_;                          //!< Number of pixel on Y-axis in the image plane
  double x_fov_per_pix_;                         //!< Field of view per pixel of X-axis in the image plane [rad/pix]
  double y_fov_per_pix_;                         //!< Field of view per pixel of Y-axis in the image plane [rad/pix]
  double x_field_of_view_rad;                    //!< Field of view of X-axis in the image plane [rad/pix]
  double y_field_of_view_rad;                    //!< Field of view of Y-axis in the image plane [rad/pix]
  double ground_position_x_image_sensor_ = 0.0;  //!< Ground position z
  double ground_position_y_image_sensor_ = 0.0;  //!< Ground position y

  bool is_sun_in_forbidden_angle = false;    //!< Is the sun in the forbidden angle
  bool is_earth_in_forbidden_angle = false;  //!< Is the earth in the forbidden angle
  bool is_moon_in_forbidden_angle = false;   //!< Is the moon in the forbidden angle

  size_t number_of_logged_stars_;  //!< Number of logged stars

  libra::Vector<2> sun_position_image_sensor{-1};    //!< Position of the sun on the image plane
  libra::Vector<2> earth_position_image_sensor{-1};  //!< Position of the earth on the image plane
  libra::Vector<2> moon_position_image_sensor{-1};   //!< Position of the moon on the image plane
  libra::Vector<3> initial_ground_position_ecef_m_;  //!< Initial spacecraft position

  std::vector<Star> star_list_in_sight;  //!< Star information in the field of view

  /**
   * @fn JudgeForbiddenAngle
   * @brief Judge the forbidden angles are violated
   * @param [in] target_b: Direction vector of target on the body fixed frame
   * @param [in] forbidden_angle: Forbidden angle [rad]
   */
  bool JudgeForbiddenAngle(const libra::Vector<3>& target_b, const double forbidden_angle);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
   */
  void MainRoutine(const int time_count);

  /**
   * @fn Observe
   * @brief Convert body fixed direction vector to position on image sensor plane
   * @param [out] position_image_sensor: Position on image sensor plane
   * @param [in] target_b: Direction vector of target on the body fixed frame
   */
  void Observe(libra::Vector<2>& position_image_sensor, const libra::Vector<3, double> target_b);
  /**
   * @fn ObserveStars
   * @brief Observe stars from Hipparcos catalogue
   */
  void ObserveStars();

  const Attitude* attitude_;                                      //!< Attitude information
  const HipparcosCatalogue* hipparcos_;                           //!< Star information
  const LocalCelestialInformation* local_celestial_information_;  //!< Local celestial information
                                                                  /*
                                                                   * @fn ObserveGroundPositionDeviation
                                                                   * @brief Calculate the deviation of the ground position from its initial value in the image sensor
                                                                   */
  void ObserveGroundPositionDeviation();

  const Orbit* orbit_;  //!< Orbit information
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
  //  libra::Vector<3> sun_pos_c;
  //  libra::Vector<3> earth_pos_c;
  //  libra::Vector<3> moon_pos_c;
  //  double angle_sun;
  //  double angle_earth;
  //  double angle_moon;
  //*************************************************************
};

/*
 * @fn InitTelescope
 * @brief Initialize function of Telescope
 * @param [in] clock_generator: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] file_name: Path to initialize file
 * @param [in] attitude: Attitude information
 * @param [in] hipparcos: Star information by Hipparcos catalogue
 * @param [in] local_celestial_information: Local celestial information
 */
Telescope InitTelescope(ClockGenerator* clock_generator, int sensor_id, const std::string file_name, const Attitude* attitude,
                        const HipparcosCatalogue* hipparcos, const LocalCelestialInformation* local_celestial_information,
                        const Orbit* orbit = nullptr);

#endif  // S2E_COMPONENTS_REAL_MISSION_TELESCOPE_HPP_P_
