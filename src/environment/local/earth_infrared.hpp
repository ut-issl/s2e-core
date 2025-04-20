/**
 * @file earth_infrared.hpp
 * @brief Class to manage earth infrared
 */

 #ifndef S2E_ENVIRONMENT_LOCAL_EARTH_INFRARED_HPP_
 #define S2E_ENVIRONMENT_LOCAL_EARTH_INFRARED_HPP_
 
 #include "environment/global/physical_constants.hpp"
 #include "environment/local/local_celestial_information.hpp"
 #include "solar_radiation_pressure_environment.hpp"
 
 namespace s2e::environment {
 
 /**
  * @class EarthInfrared
  * @brief Class to calculate Earth Infrared
  */
 class EarthInfrared : public logger::ILoggable {
  public:
   /**
    * @fn EarthInfrared
    * @brief Constructor
    * @param [in] local_celestial_information: Local celestial information
    */
   EarthInfrared(LocalCelestialInformation* local_celestial_information, SolarRadiationPressureEnvironment* srp_environment);
 
   /**
    * @fn ~EarthInfrared
    * @brief Destructor
    */
   virtual ~EarthInfrared() {}
 
   /**
    * @fn UpdateAllStates
    * @brief Update earth infrared
    */
   void UpdateAllStates();
 
   // Getter
   /**
    * @fn GetPowerDensity_W_m2
    * @brief Calculate and return earth infrared [W/m^2]
    */
   inline double GetEarthInfraredRadiationPower_W_m2() const { return earth_infrared_W_m2_; }
   /**
    * @fn GetIsEclipsed
    * @brief Returns true if the shadow function is less than 1
    */
   inline bool GetIsCalcEarthInfraredEnabled() const { return is_calc_earth_infrared_enabled_; }
 
   // Setter
   /**
    * @fn SetIsCalcEarthInfraredEnabled
    * @brief Set calculation flag
    * @param [in] is_calc_earth_infrared_enabled: Calculation flag
    */
   inline void SetIsCalcEarthInfraredEnabled(const bool is_calc_earth_infrared_enabled) {
     is_calc_earth_infrared_enabled_ = is_calc_earth_infrared_enabled;
   }
   inline void SetEarthTempHotSide(const double earth_infrared_temperature_hot_side) {
     earth_infrared_temperature_hot_side_ = earth_infrared_temperature_hot_side;
   }
   inline void SetEarthTempColdSide(const double earth_infrared_temperature_cold_side) {
     earth_infrared_temperature_cold_side_ = earth_infrared_temperature_cold_side;
   }
 
  private:
   double earth_infrared_W_m2_ = 0.0;                   //!< Earth infrared [W/m^2]
   bool is_calc_earth_infrared_enabled_ = false;        //!< Calculation flag
   double earth_infrared_temperature_hot_side_ = 0.0;   //!< Earth infrared temperature hot side [K]
   double earth_infrared_temperature_cold_side_ = 0.0;  //!< Earth infrared temperature cold side [K]
 
   LocalCelestialInformation* local_celestial_information_;  //!< Local celestial information
   SolarRadiationPressureEnvironment* srp_environment_;      //!< Solar radiation pressure environment
 
   // Override logger::ILoggable
   /**
    * @fn GetLogHeader
    * @brief Override GetLogHeader function of logger::ILoggable
    */
   virtual std::string GetLogHeader() const;
   /**
    * @fn GetLogValue
    * @brief Override GetLogValue function of logger::ILoggable
    */
   virtual std::string GetLogValue() const;
 
   /**
    * @fn CalcEarthInfrared
    * @brief Calculate earth infrared
    * @param [in] local_celestial_information: Local celestial information
    */
   void CalcEarthInfrared(const LocalCelestialInformation* local_celestial_information);
 };
 
 /**
  * @fn InitEarthInfrared
  * @brief Initialize earth infrared
  * @param [in] initialize_file_path: Path to initialize file
  */
 EarthInfrared InitEarthInfrared(std::string initialize_file_path, LocalCelestialInformation* local_celestial_information,
                                 SolarRadiationPressureEnvironment* srp_environment);
 
 }  // namespace s2e::environment
 
 #endif  // S2E_ENVIRONMENT_LOCAL_EARTH_INFRARED_HPP_