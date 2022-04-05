#ifndef __SRPEnvironment_h__
#define __SRPEnvironment_h__
#include <Environment/Local/LocalCelestialInformation.h>
#include <Interface/LogOutput/ILoggable.h>

#include <Library/math/Vector.hpp>

using libra::Vector;

class SRPEnvironment : public ILoggable {
 public:
  bool IsCalcEnabled = true;

  SRPEnvironment(LocalCelestialInformation* local_celes_info);  // Default constructor
  void UpdateAllStates();
  void UpdatePressure();
  double CalcTruePressure() const;      // Obtaining solar radiation pressure that
                                        // takes into account eclipse [N/m^2]
  double CalcPowerDensity() const;      // Get solar power per unit area considering eclipse [W/m^2]
  double GetPressure() const;           // Get solar pressure without eclipse effect [N/m^2]
  double GetSolarConstant() const;      // Get solar constant value [W/m^2]
  double GetShadowCoefficient() const;  // Get Shadow function
  inline bool GetIsEclipsed() const { return (shadow_coefficient_ >= 1.0 ? false : true); }  // Returns true if the shadow function is less than 1

  virtual std::string GetLogHeader() const;  // log of header
  virtual std::string GetLogValue() const;   // log of value

 private:
  double pressure_;                  // solar radiation constant [N/m^2]
  double solar_constant_;            // solar constant [W/m^2]
  double shadow_coefficient_ = 1.0;  // shadow function
  double sun_radius_m_;
  std::string shadow_source_name_;

  LocalCelestialInformation* local_celes_info_;

  void CalcShadowCoefficient(std::string shadow_source_name);
};

#endif /* SRPEnvironment_h */
