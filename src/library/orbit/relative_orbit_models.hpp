/**
 * @file relative_orbit_models.hpp
 * @brief Functions for relative orbit
 */

#ifndef S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_MODEL_HPP_
#define S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_MODEL_HPP_

#include "../math/matrix.hpp"
#include "../math/vector.hpp"

/**
 * @enum RelativeOrbitModel
 * @brief Relative orbit model
 */
enum class RelativeOrbitModel { kHill = 0 };

/**
 * @enum StmModel
 * @brief State Transition Matrix for the relative orbit
 */
enum class StmModel { kHcw = 0 };

// Dynamics Models
/**
 * @fn CalcHillSystemMatrix
 * @brief Calculate Hill System Matrix
 * @param [in] orbit_radius_m: Orbit radius [m]
 * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
 * @return System matrix
 */
libra::Matrix<6, 6> CalcHillSystemMatrix(const double orbit_radius_m, const double gravity_constant_m3_s2);

// STMs
/**
 * @fn CalcHcwStm
 * @brief Calculate HCW State Transition Matrix
 * @param [in] orbit_radius_m: Orbit radius [m]
 * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
 * @param [in] elapsed_time_s: Elapsed time [s]
 * @return State Transition Matrix
 */
libra::Matrix<6, 6> CalcHcwStm(const double orbit_radius_m, const double gravity_constant_m3_s2, const double elapsed_time_s);

#endif  // S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_MODEL_HPP_
