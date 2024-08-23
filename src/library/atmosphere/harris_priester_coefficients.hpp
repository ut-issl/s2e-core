/**
 * @file harris_priester_coefficients.hpp
 * @brief Harris-Priester coefficients
 */
#ifndef S2E_LIBRARY_HARRIS_PRIESTER_COEFFICIENTS_HPP_
#define S2E_LIBRARY_HARRIS_PRIESTER_COEFFICIENTS_HPP_

#include <map>

namespace libra::atmosphere {

// Height [km], density [g/km3]
// TODO: Add other solar activities value
const std::map<double, double> harris_priester_min_density_table = {
    {100, 497400.0}, {120, 24900.0},  {130, 8377.0},  {140, 3899.0},   {150, 2122.0},   {160, 1263.0},   {170, 800.8},    {180, 528.3},
    {190, 361.7},    {200, 255.7},    {210, 183.9},   {220, 134.1},    {230, 99.49},    {240, 74.88},    {250, 57.09},    {260, 44.03},
    {270, 34.30},    {280, 26.97},    {290, 21.39},   {300, 17.08},    {320, 10.99},    {340, 7.214},    {360, 4.824},    {380, 3.274},
    {400, 2.249},    {420, 1.558},    {440, 1.091},   {460, 0.7701},   {480, 0.5474},   {500, 0.3916},   {520, 0.2819},   {540, 0.2042},
    {560, 0.1488},   {580, 0.1092},   {600, 0.08070}, {620, 0.06012},  {640, 0.04519},  {660, 0.03430},  {680, 0.02632},  {700, 0.02043},
    {720, 0.01607},  {740, 0.01281},  {760, 0.01036}, {780, 0.008496}, {800, 0.007069}, {840, 0.004680}, {880, 0.003200}, {920, 0.002210},
    {960, 0.001560}, {1000, 0.001150}};

const std::map<double, double> harris_priester_max_density_table = {
    {100, 497400.0}, {120, 24900.0}, {130, 8710.0},  {140, 4059.0},  {150, 2215.0},  {160, 1344.0}, {170, 875.8},  {180, 601.0},   {190, 429.7},
    {200, 316.2},    {210, 239.6},   {220, 185.3},   {230, 145.5},   {240, 115.7},   {250, 93.08},  {260, 75.55},  {270, 61.82},   {280, 50.95},
    {290, 42.26},    {300, 35.26},   {320, 25.11},   {340, 18.19},   {360, 13.37},   {380, 9.955},  {400, 7.492},  {420, 5.684},   {440, 4.355},
    {460, 3.362},    {480, 2.612},   {500, 2.042},   {520, 1.605},   {540, 1.267},   {560, 1.005},  {580, 0.7997}, {600, 0.6390},  {620, 0.5123},
    {640, 0.4121},   {660, 0.3325},  {680, 0.2691},  {700, 0.2185},  {720, 0.1779},  {740, 0.1452}, {760, 0.1190}, {780, 0.09776}, {800, 0.08059},
    {840, 0.05741},  {880, 0.04210}, {920, 0.03130}, {960, 0.02360}, {1000, 0.01810}};

}  // namespace libra::atmosphere

#endif  // S2E_LIBRARY_HARRIS_COEFFICIENTS_HPP_