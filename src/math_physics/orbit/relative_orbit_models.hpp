/**
 * @file relative_orbit_models.hpp
 * @brief Functions for relative orbit
 */

#ifndef S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_MODEL_HPP_
#define S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_MODEL_HPP_

#include "../math/matrix.hpp"
#include "../math/vector.hpp"
#include "../../dynamics/orbit/orbit.hpp"
#include "./orbital_elements.hpp"

namespace orbit {

/**
 * @enum RelativeOrbitModel
 * @brief Relative orbit model
 */
enum class RelativeOrbitModel { kHill = 0 };

/**
 * @enum StmModel
 * @brief State Transition Matrix for the relative orbit
 */
enum class StmModel { kHcw = 0, kMelton = 1, kSs = 2, kSabatini = 3, kCarter = 4, kYamakawaAnkersen = 5 };

// Dynamics Models
/**
 * @fn CalcHillSystemMatrix
 * @brief Calculate Hill System Matrix
 * @param [in] orbit_radius_m: Orbit radius [m]
 * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
 * @return System matrix
 */
math::Matrix<6, 6> CalcHillSystemMatrix(const double orbit_radius_m, const double gravity_constant_m3_s2);

// STMs
/**
 * @fn CalcHcwStm
 * @brief Calculate HCW State Transition Matrix
 * @param [in] orbit_radius_m: Orbit radius [m]
 * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
 * @param [in] elapsed_time_s: Elapsed time [s]
 * @return State Transition Matrix
 */
math::Matrix<6, 6> CalcHcwStm(const double orbit_radius_m, const double gravity_constant_m3_s2, const double elapsed_time_s);

/**
 * @fn CalcMeltonStm
 * @brief Calculate Melton State Transition Matrix
 * @param [in] orbit_radius_m: Orbit radius [m]
 * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
 * @param [in] elapsed_time_s: Elapsed time [s]
 * @param [in] reference_oe: Orbital elements of reference satellite
 * @return State Transition Matrix
 */
math::Matrix<6, 6> CalcMeltonStm(double orbit_radius_m, double gravity_constant_m3_s2, double elapsed_time_s, OrbitalElements* reference_oe);

/**
 * @fn CalcSsStm
 * @brief Calculate SS State Transition Matrix
 * @param [in] orbit_radius_m: Orbit radius [m]
 * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
 * @param [in] elapsed_time_s: Elapsed time [s]
 * @param [in] reference_oe: Orbital elements of reference satellite
 * @return State Transition Matrix
 */
math::Matrix<6, 6> CalcSsStm(double orbit_radius_m, double gravity_constant_m3_s2, double elapsed_time_s, OrbitalElements* reference_oe);

/**
 * @fn CalcSsCorrectionTerm
 * @brief Calculate SS Correction Term
 * @param [in] orbit_radius_m: Orbit radius [m]
 * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
 * @param [in] elapsed_time_s: Elapsed time [s]
 * @param [in] reference_oe: Orbital elements of reference satellite
 * @return State Transition Matrix
 */
math::Vector<6> CalcSsCorrectionTerm(double orbit_radius_m, double gravity_constant_m3_s2, double elapsed_time_s, OrbitalElements* reference_oe);

/**
 * @fn CalcSabatiniStm
 * @brief Calculate Sabatani State Transition Matrix
 * @param [in] orbit_radius_m: Orbit radius [m]
 * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
 * @param [in] elapsed_time_s: Elapsed time [s]
 * @param [in] reference_oe: Orbital elements of reference satellite
 * @return State Transition Matrix
 */
math::Matrix<6, 6> CalcSabatiniStm(double orbit_radius_m, double gravity_constant_m3_s2, double elapsed_time_s, double f_ref_rad, math::Vector<3> position_i_m_s, math::Vector<3> velocity_i_m_s, OrbitalElements* reference_oe);

/**
 * @fn CalcCarterStm
 * @brief Calculate Carter State Transition Matrix
 * @param [in] orbit_radius_m: Orbit radius [m]
 * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
 * @param [in] f_ref_rad: True anomaly of the reference satellite [rad]
 * @param [in] reference_oe: Orbital elements of reference satellite
 * @return State Transition Matrix
 */
math::Matrix<6, 6> CalcCarterStm(double orbit_radius_m, double gravity_constant_m3_s2, double f_ref_rad, OrbitalElements* reference_oe);

}  // namespace orbit

#endif  // S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_MODEL_HPP_
