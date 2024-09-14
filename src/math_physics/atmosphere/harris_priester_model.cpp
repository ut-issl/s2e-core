/**
 * @file harris_priester_model.cpp
 * @brief Harris-Priester earth's atmospheric density model
 */
#include "harris_priester_model.hpp"

#include <cmath>
#include <math_physics/math/constants.hpp>
#include <utilities/macros.hpp>

#include "harris_priester_coefficients.hpp"

namespace atmosphere {

/**
 * @fn CalcScaleHeight_km
 * @brief Calculate scale height
 * @param [in] density_itr: Iterator for density map table
 * @return Scale height [km]
 */
double CalcScaleHeight_km(const std::map<double, double>::const_iterator density_itr);
/**
 * @fn CalcApexDensity_g_km3
 * @brief Calculate Apex or Antapex density
 * @param [in] density_itr: Iterator for density map table
 * @param [in] altitude_km: Altitude [km]
 * @return Apex or Antapex density
 */
double CalcApexDensity_g_km3(const std::map<double, double>::const_iterator density_itr, const double altitude_km);

double CalcAirDensityWithHarrisPriester_kg_m3(const geodesy::GeodeticPosition geodetic_position, const s2e::math::Vector<3> sun_direction_eci,
                                              const double f10_7, const double exponent_parameter) {
  // Altitude
  double altitude_km = geodetic_position.GetAltitude_m() / 1000.0;

  // Position
  s2e::math::Vector<3> position_ecef_m = geodetic_position.CalcEcefPosition();

  // Phi: angle between the satellite position and apex of the diurnal bulge
  double sun_ra_rad;   //!< Right ascension of the sun phi
  double sun_dec_rad;  //!< Declination of the sun theta
  sun_ra_rad = atan2(sun_direction_eci[1], sun_direction_eci[0]);
  sun_dec_rad = atan2(sun_direction_eci[2], sqrt(sun_direction_eci[0] * sun_direction_eci[0] + sun_direction_eci[1] * sun_direction_eci[1]));
  s2e::math::Vector<3> apex_direction;
  const double lag_angle_rad = 30.0 * s2e::math::deg_to_rad;
  double apex_ra_rad = sun_ra_rad + lag_angle_rad;
  apex_direction[0] = cos(sun_dec_rad) * cos(apex_ra_rad);
  apex_direction[1] = cos(sun_dec_rad) * sin(apex_ra_rad);
  apex_direction[2] = sin(sun_dec_rad);

  double beta_rad = s2e::math::InnerProduct(position_ecef_m.CalcNormalizedVector(), apex_direction);
  double cos_phi = pow(0.5 + beta_rad / 2.0, exponent_parameter / 2.0);

  // Find density coefficients from altitude
  UNUSED(f10_7);  // TODO: Use F10.7 value to search coefficients
  auto min_density_itr = harris_priester_min_density_table.upper_bound(altitude_km);
  auto max_density_itr = harris_priester_max_density_table.upper_bound(altitude_km);
  if (min_density_itr == harris_priester_min_density_table.begin()) return min_density_itr->second;
  if (min_density_itr == harris_priester_min_density_table.end()) return (min_density_itr->second + max_density_itr->second) / 2.0;
  min_density_itr = prev(min_density_itr);
  max_density_itr = prev(max_density_itr);

  // Calculate density
  double antapex_density_g_km3 = CalcApexDensity_g_km3(min_density_itr, altitude_km);
  double apex_density_g_km3 = CalcApexDensity_g_km3(max_density_itr, altitude_km);

  double density_g_km3 = antapex_density_g_km3 + (apex_density_g_km3 - antapex_density_g_km3) * cos_phi;

  return density_g_km3 * 1e-12;  // Unit conversion g/km3 -> kg/m^3
}

double CalcScaleHeight_km(const std::map<double, double>::const_iterator density_itr) {
  const double height_1_km = density_itr->first;
  const double density_1_g_km3 = density_itr->second;
  const double height_2_km = next(density_itr)->first;
  const double density_2_g_km3 = next(density_itr)->second;

  double scale_height_km = (height_1_km - height_2_km) / log(density_2_g_km3 / density_1_g_km3);
  return scale_height_km;
}

double CalcApexDensity_g_km3(const std::map<double, double>::const_iterator density_itr, const double altitude_km) {
  const double scale_height_km = CalcScaleHeight_km(density_itr);
  const double height_km = density_itr->first;
  const double density_g_km3 = density_itr->second;

  double apex_density_g_km3 = density_g_km3 * exp((height_km - altitude_km) / scale_height_km);
  return apex_density_g_km3;
}

}  // namespace atmosphere
