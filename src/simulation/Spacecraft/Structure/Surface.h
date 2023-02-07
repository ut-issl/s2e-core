/**
 * @file Surface.h
 * @brief Definition of spacecraft surface
 */

#pragma once

#include <Library/math/Vector.hpp>
using libra::Vector;

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
  Surface(Vector<3> position, Vector<3> normal, double area, double reflectivity, double specularity, double air_specularity);
  /**
   * @fn ~Surface
   * @brief Destructor
   */
  ~Surface(){};

  // Getter
  /**
   * @fn GetPosition
   * @brief Return position vector of geometric center of the surface in body frame and meter unit
   */
  inline const Vector<3>& GetPosition(void) const { return position_; }
  /**
   * @fn GetNormal
   * @brief Return normal vector of the surface in body frame
   */
  inline const Vector<3>& GetNormal(void) const { return normal_; }
  /**
   * @fn GetArea
   * @brief Return area of the surface in meter^2 unit
   */
  inline const double& GetArea(void) const { return area_; }
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
  inline void SetPosition_b_m(const Vector<3> position_b_m) { position_ = position_b_m; }
  /**
   * @fn SetNormal
   * @brief Set normal vector of the surface in body frame
   * @param[in] normal_b: Normal vector of the surface in body frame
   */
  inline void SetNormal_b(const Vector<3> normal_b) {
    normal_ = normal_b;
    normal_ = normalize(normal_);
  }
  /**
   * @fn SetArea
   * @brief Set area of the surface
   * @param[in] area_m2: Area of the surface [m2]
   */
  inline void SetArea(const double area_m2) {
    if (area_m2 > 0.0) area_ = area_m2;
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
  Vector<3> position_;      //!< Position vector of the surface @ Body Frame [m]
  Vector<3> normal_;        //!< Normal unit vector of the surface @ Body Frame [-]
  double area_;             //!< Area of the surface [m2]
  double reflectivity_;     //!< Total reflectivity for solar wavelength (1.0 - solar absorption)
  double specularity_;      //!< Ratio of specular reflection in the total reflected light
  double air_specularity_;  //!< Specularity for air drag
};
