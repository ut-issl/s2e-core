/**
 * @file simple_air_density_model.cpp
 * @brief Standard earth's atmospheric density model
 */
#include "simple_air_density_model.hpp"

#include <cmath>

namespace libra::atmosphere {

double CalcAirDensityWithSimpleModel(const double altitude_m) {
  double altitude_km = altitude_m / 1000.0;
  double scale_height_km;
  double base_height_km;
  double base_rho_kg_m3;

  // scale_height_km values: Ref "ミッション解析と軌道設計の基礎" (in Japanese)
  if (altitude_km > 1000.0) {
    scale_height_km = 268.0;
    base_height_km = 1000.0;
    base_rho_kg_m3 = 3.019E-15;
  } else if (altitude_km >= 900.0 && altitude_km < 1000.0) {
    scale_height_km = 181.05;
    base_height_km = 900.0;
    base_rho_kg_m3 = 5.245E-15;
  } else if (altitude_km >= 800.0 && altitude_km < 900.0) {
    scale_height_km = 124.64;
    base_height_km = 800.0;
    base_rho_kg_m3 = 1.170E-14;
  } else if (altitude_km >= 700.0 && altitude_km < 800.0) {
    scale_height_km = 88.667;
    base_height_km = 700.0;
    base_rho_kg_m3 = 3.614E-14;
  } else if (altitude_km >= 600.0 && altitude_km < 700.0) {
    scale_height_km = 71.835;
    base_height_km = 600.0;
    base_rho_kg_m3 = 1.454E-13;
  } else if (altitude_km >= 500.0 && altitude_km < 600.0) {
    scale_height_km = 63.822;
    base_height_km = 500.0;
    base_rho_kg_m3 = 6.967E-13;
  } else if (altitude_km >= 450.0 && altitude_km < 500.0) {
    scale_height_km = 60.828;
    base_height_km = 450.0;
    base_rho_kg_m3 = 1.585E-12;
  } else if (altitude_km >= 400.0 && altitude_km < 450.0) {
    scale_height_km = 58.515;
    base_height_km = 400.0;
    base_rho_kg_m3 = 3.725E-12;
  } else if (altitude_km >= 350.0 && altitude_km < 400.0) {
    scale_height_km = 53.298;
    base_height_km = 350.0;
    base_rho_kg_m3 = 9.158E-12;
  } else if (altitude_km >= 300.0 && altitude_km < 350.0) {
    scale_height_km = 53.628;
    base_height_km = 300.0;
    base_rho_kg_m3 = 2.418E-11;
  } else if (altitude_km >= 250.0 && altitude_km < 300.0) {
    scale_height_km = 45.546;
    base_height_km = 250.0;
    base_rho_kg_m3 = 7.248E-11;
  } else if (altitude_km >= 200.0 && altitude_km < 250.0) {
    scale_height_km = 37.105;
    base_height_km = 200.0;
    base_rho_kg_m3 = 2.789E-10;
  } else if (altitude_km >= 180.0 && altitude_km < 200.0) {
    scale_height_km = 29.740;
    base_height_km = 180.0;
    base_rho_kg_m3 = 5.464E-10;
  } else if (altitude_km >= 150.0 && altitude_km < 180.0) {
    scale_height_km = 22.523;
    base_height_km = 150.0;
    base_rho_kg_m3 = 2.070E-9;
  } else if (altitude_km >= 140.0 && altitude_km < 150.0) {
    scale_height_km = 16.149;
    base_height_km = 140.0;
    base_rho_kg_m3 = 3.845E-9;
  } else if (altitude_km >= 130.0 && altitude_km < 140.0) {
    scale_height_km = 12.636;
    base_height_km = 130.0;
    base_rho_kg_m3 = 8.484E-9;
  } else if (altitude_km >= 120.0 && altitude_km < 130.0) {
    scale_height_km = 9.473;
    base_height_km = 120.0;
    base_rho_kg_m3 = 2.438E-8;
  } else if (altitude_km >= 110.0 && altitude_km < 120.0) {
    scale_height_km = 7.263;
    base_height_km = 110.0;
    base_rho_kg_m3 = 9.661E-8;
  } else if (altitude_km >= 100.0 && altitude_km < 110.0) {
    scale_height_km = 5.877;
    base_height_km = 100.0;
    base_rho_kg_m3 = 5.297E-7;
  } else if (altitude_km >= 90.0 && altitude_km < 100.0) {
    scale_height_km = 5.382;
    base_height_km = 90.0;
    base_rho_kg_m3 = 3.396E-6;
  } else if (altitude_km >= 80.0 && altitude_km < 90.0) {
    scale_height_km = 5.799;
    base_height_km = 80.0;
    base_rho_kg_m3 = 1.905E-5;
  } else if (altitude_km >= 70.0 && altitude_km < 80.0) {
    scale_height_km = 6.549;
    base_height_km = 70.0;
    base_rho_kg_m3 = 8.770E-5;
  } else if (altitude_km >= 60.0 && altitude_km < 70.0) {
    scale_height_km = 7.714;
    base_height_km = 60.0;
    base_rho_kg_m3 = 3.206E-4;
  } else if (altitude_km >= 50.0 && altitude_km < 60.0) {
    scale_height_km = 8.382;
    base_height_km = 50.0;
    base_rho_kg_m3 = 1.057E-3;
  } else if (altitude_km >= 40.0 && altitude_km < 50.0) {
    scale_height_km = 7.554;
    base_height_km = 40.0;
    base_rho_kg_m3 = 3.972E-3;
  } else if (altitude_km >= 30.0 && altitude_km < 40.0) {
    scale_height_km = 6.682;
    base_height_km = 30.0;
    base_rho_kg_m3 = 1.774E-2;
  } else if (altitude_km >= 25.0 && altitude_km < 30.0) {
    scale_height_km = 6.349;
    base_height_km = 25.0;
    base_rho_kg_m3 = 3.899E-2;
  } else if (altitude_km >= 0.0 && altitude_km < 25.0) {
    scale_height_km = 7.249;
    base_height_km = 0.0;
    base_rho_kg_m3 = 1.225;
  } else {  // In case of altitude_km is minus value
    scale_height_km = 7.249;
    base_height_km = 0.0;
    base_rho_kg_m3 = 0.0;
    return 0.0;
  }

  double rho_kg_m3 = base_rho_kg_m3 * exp(-(altitude_km - base_height_km) / scale_height_km);
  return rho_kg_m3;
}

}  // namespace libra::atmosphere
