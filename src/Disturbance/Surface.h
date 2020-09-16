#pragma once

#include "../Library/math/Vector.hpp"
using libra::Vector;

class Surface
{
public:
  Surface(Vector<3> position, Vector<3> normal, double area, double reflectivity, double specularity, double air_specularity);
  ~Surface(){};
  inline const Vector<3>& GetPosition(void) const {return position_;}
  inline const Vector<3>& GetNormal(void) const {return normal_;}
  inline const double& GetArea(void) const {return area_;}
  inline const double& GetReflectivity(void) const {return reflectivity_;}
  inline const double& GetSpecularity(void) const {return specularity_;}
  inline const double& GetAirSpecularity(void) const {return air_specularity_;}
private:
  Vector<3> position_;//Position vector of the surfaces @ Body Frame
  Vector<3> normal_;  //Normal unit vector of the surfaces @ Body Frame
  double area_;
  double reflectivity_; // 1.0 - solar absorption
  double specularity_;  // Ratio of specular reflection in the total reflected light
  double air_specularity_;  //Specularity for air drag
};