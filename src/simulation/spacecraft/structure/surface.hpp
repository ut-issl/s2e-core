/**
 * @file surface.hpp
 * @brief Definition of spacecraft surface
 */

#ifndef S2E_SIMULATION_SPACECRAFT_STRUCTURE_SURFACE_HPP_
#define S2E_SIMULATION_SPACECRAFT_STRUCTURE_SURFACE_HPP_

#include <math_physics/math/vector.hpp>

namespace s2e::simulation {

/**
 * @class Surface
 * @brief Class for spacecraft surface
 */
class Surface {
 public:
  /**
   * @fn Surface
   * @brief Constructor
   */
  Surface(const s2e::math::Vector<3> position_b_m, const s2e::math::Vector<3> normal_b, const double area_m2, const double reflectivity,
          const double specularity, const double air_specularity);
  /**
   * @fn ~Surface
   * @brief Destructor
   */
  ~Surface(){};

  // Getter
  /**
   * @fn GetPosition_b_m
   * @brief Return position vector of geometric center of the surface in body frame and meter unit
   */
  inline const s2e::math::Vector<3>& GetPosition_b_m(void) const { return position_b_m_; }
  /**
   * @fn GetNormal_b
   * @brief Return normal vector of the surface in body frame
   */
  inline const s2e::math::Vector<3>& GetNormal_b(void) const { return normal_b_; }
  /**
   * @fn GetArea_m2
   * @brief Return area of the surface in meter^2 unit
   */
  inline const double& GetArea_m2(void) const { return area_m2_; }
  /**
   * @fn GetReflectivity
   * @brief Return reflectivity of the surface
   */
  inline const double& GetReflectivity(void) const { return reflectivity_; }
  /**
   * @fn GetSpecularity(
   * @brief Return specularity of the surface
   */
  inline const double& GetSpecularity(void) const { return specularity_; }
  /**
   * @fn GetAirSpecularity
   * @brief Return specularity of air drag of the surface
   */
  inline const double& GetAirSpecularity(void) const { return air_specularity_; }

  // Setter
  /**
   * @fn SetPosition
   * @brief Set position vector of geometric center of the surface in body frame [m]
   * @param[in] position_b_m: Position vector of geometric center of the surface in body frame [m]
   */
  inline void SetPosition_b_m(const s2e::math::Vector<3> position_b_m) { position_b_m_ = position_b_m; }
  /**
   * @fn SetNormal
   * @brief Set normal vector of the surface in body frame
   * @param[in] normal_b: Normal vector of the surface in body frame
   */
  inline void SetNormal_b(const s2e::math::Vector<3> normal_b) { normal_b_ = normal_b.CalcNormalizedVector(); }
  /**
   * @fn SetArea_m2
   * @brief Set area of the surface
   * @param[in] area_m2: Area of the surface [m2]
   */
  inline void SetArea_m2(const double area_m2) {
    if (area_m2 > 0.0) area_m2_ = area_m2;
  }
  /**
   * @fn SetReflectivity
   * @brief Set reflectivity of the surface
   * @param[in] reflectivity: Reflectivity of the surface
   */
  inline void SetReflectivity(const double reflectivity) {
    if (reflectivity >= 0.0 && reflectivity <= 1.0) reflectivity_ = reflectivity;
  }
  /**
   * @fn SetSpecularity
   * @brief Set specularity of the surface
   * @param[in] specularity: Specularity of the surface
   */
  inline void SetSpecularity(const double specularity) {
    if (specularity >= 0.0 && specularity <= 1.0) specularity_ = specularity;
  }
  /**
   * @fn SetAirSpecularity
   * @brief Set air specularity of the surface
   * @param[in] air_specularity: Air specularity of the surface
   */
  inline void SetAirSpecularity(const double air_specularity) {
    if (air_specularity >= 0.0 && air_specularity <= 1.0) air_specularity_ = air_specularity;
  }

 private:
  s2e::math::Vector<3> position_b_m_;  //!< Position vector of the surface @ Body Frame [m]
  s2e::math::Vector<3> normal_b_;      //!< Normal unit vector of the surface @ Body Frame [-]
  double area_m2_;                //!< Area of the surface [m2]
  double reflectivity_;           //!< Total reflectivity for solar wavelength (1.0 - solar absorption)
  double specularity_;            //!< Ratio of specular reflection in the total reflected light
  double air_specularity_;        //!< Specularity for air drag
};

} // namespace s2e::simulation

#endif  // S2E_SIMULATION_SPACECRAFT_STRUCTURE_SURFACE_HPP_
