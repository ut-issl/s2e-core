/**
 * @file harris_priester_model.hpp
 * @brief Harris-Priester earth's atmospheric density model
 * @note Ref: O. Montenbruck and E. Gill, Satellite Orbits, Chp. 3.5
 */
#ifndef S2E_LIBRARY_HARRIS_PRIESTER_MODEL_HPP_
#define S2E_LIBRARY_HARRIS_PRIESTER_MODEL_HPP_

#include <math_physics/geodesy/geodetic_position.hpp>
#include <math_physics/math/vector.hpp>

namespace s2e::atmosphere {

/**
 * @fn CalcAirDensityWithHarrisPriester
 * @brief Calculate atmospheric density with Harris-Priester method
 * @param [in] geodetic_position: Spacecraft geodetic position
 * @param [in] sun_direction_eci: Sun direction unit vector in ECI frame
 * @param [in] f10_7: F10.7 radiation index (not used now)
 * @param [in] exponent_parameter: n in the equation. n=2 for low inclination orbit and n=6 for polar orbit.
 * @return Atmospheric density [kg/m^3]
 */
double CalcAirDensityWithHarrisPriester_kg_m3(const geodesy::GeodeticPosition geodetic_position, const math::Vector<3> sun_direction_eci,
                                              const double f10_7 = 100.0, const double exponent_parameter = 4);

}  // namespace s2e::atmosphere

#endif  // S2E_LIBRARY_HARRIS_PRIESTER_HPP_
