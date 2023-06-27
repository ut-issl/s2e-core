/**
 * @file harris_priester_model.hpp
 * @brief Harris-Priester earth's atmospheric density model
 */
#ifndef S2E_LIBRARY_HARRIS_PRIESTER_MODEL_HPP_
#define S2E_LIBRARY_HARRIS_PRIESTER_MODEL_HPP_

#include <library/geodesy/geodetic_position.hpp>
#include <library/math/vector.hpp>

namespace libra::atmosphere {

/**
 * @fn CalcAirDensityWithHarrisPriester
 * @brief Calculate atmospheric density with Harris-Priester method
 * @param [in]
 * @return Atmospheric density [kg/m^3]
 */
double CalcAirDensityWithHarrisPriester_kg_m3(const GeodeticPosition geodetic_position, const libra::Vector<3> sun_direction_i,
                                              const double exponent_parameter = 4);

}  // namespace libra::atmosphere

#endif  // S2E_LIBRARY_HARRIS_PRIESTER_HPP_
