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

 private:
  Vector<3> position_;      //!< Position vector of the surface @ Body Frame [m]
  Vector<3> normal_;        //!< Normal unit vector of the surface @ Body Frame [-]
  double area_;             //!< Area of the surface [m2]
  double reflectivity_;     //!< Total reflectivity for solar wavelength (1.0 - solar absorption)
  double specularity_;      //!< Ratio of specular reflection in the total reflected light
  double air_specularity_;  //!< Specularity for air drag
};
