/**
 * @file simple_air_density_model.hpp
 * @brief Simple earth's atmospheric density model
 */
#ifndef S2E_LIBRARY_ATMOSPHERE_SIMPLE_AIR_DENSITY_MODEL_HPP_
#define S2E_LIBRARY_ATMOSPHERE_SIMPLE_AIR_DENSITY_MODEL_HPP_

namespace libra::atmosphere {

/**
 * @fn CalcAirDensityWithSimpleModel
 * @brief Calculate atmospheric density with simplest method
 * @param [in] altitude_m: Altitude of spacecraft [m]
 * @return Atmospheric density [kg/m^3]
 */
double CalcAirDensityWithSimpleModel(const double altitude_m);

}  // namespace libra::atmosphere

#endif  // S2E_LIBRARY_ATMOSPHERE_SIMPLE_AIR_DENSITY_MODEL_HPP_
