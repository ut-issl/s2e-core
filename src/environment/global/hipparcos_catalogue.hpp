/**
 *@file hipparcos_catalogue.hpp
 *@brief Class to calculate star direction with Hipparcos catalogue
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_HIPPARCOS_CATALOGUE_HPP_
#define S2E_ENVIRONMENT_GLOBAL_HIPPARCOS_CATALOGUE_HPP_

#include <vector>

#include "logger/loggable.hpp"
#include "math_physics/math/quaternion.hpp"
#include "math_physics/math/vector.hpp"

namespace s2e::environment {

/**
 *@struct HipparcosData
 *@brief Hipparcos catalogue data
 */
struct HipparcosData {
  int hipparcos_id;            //!< Hipparcos number
  double visible_magnitude;    //!< Visible magnitude
  double right_ascension_deg;  //!< Right ascension [deg]
  double declination_deg;      //!< Declination [deg]
};
/**
 *@class HipparcosCatalogue
 *@brief Class to calculate star direction with Hipparcos catalogue
 */
class HipparcosCatalogue : public ILoggable {
 public:
  /**
   *@fn HipparcosCatalogue
   *@brief Constructor
   *@param [in] max_magnitude: Maximum star magnitude managed in this class
   *@param [in] catalogue_path: Path to Hipparcos catalogue file
   */
  HipparcosCatalogue(double max_magnitude, std::string catalogue_path);
  /**
   *@fn ~HipparcosCatalogue
   *@brief Destructor
   */
  virtual ~HipparcosCatalogue();
  /**
   *@fn ReadContents
   *@brief Read Hipparcos catalogue file
   *@param [in] file_name: Path to Hipparcos catalogue file
   *@param [in] delimiter: Delimiter for the catalogue file
   */
  bool ReadContents(const std::string& file_name, const char delimiter);

  /**
   *@fn GetCatalogueSize
   *@brief Return read catalogue size
   */
  size_t GetCatalogueSize() const { return hipparcos_catalogue_.size(); }
  /**
   *@fn GetHipparcosId
   *@brief Return Hipparcos ID of a star
   *@param [in] rank: Rank of star magnitude in read catalogue
   */
  int GetHipparcosId(size_t rank) const { return hipparcos_catalogue_[rank].hipparcos_id; }
  /**
   *@fn GetVisibleMagnitude
   *@brief Return magnitude in visible wave length of a star
   *@param [in] rank: Rank of star magnitude in read catalogue
   */
  double GetVisibleMagnitude(size_t rank) const { return hipparcos_catalogue_[rank].visible_magnitude; }
  /**
   *@fn GetRightAscension_deg
   *@brief Return right ascension of a star
   *@param [in] rank: Rank of star magnitude in read catalogue
   */
  double GetRightAscension_deg(size_t rank) const { return hipparcos_catalogue_[rank].right_ascension_deg; }
  /**
   *@fn GetDeclination_deg
   *@brief Return declination of a star
   *@param [in] rank: Rank of star magnitude in read catalogue
   */
  double GetDeclination_deg(size_t rank) const { return hipparcos_catalogue_[rank].declination_deg; }
  /**
   *@fn GetStarDir_i
   *@brief Return direction vector of a star in the inertial frame
   *@param [in] rank: Rank of star magnitude in read catalogue
   */
  s2e::math::Vector<3> GetStarDirection_i(size_t rank) const;
  /**
   *@fn GetStarDir_b
   *@brief Return direction vector of a star in the body-fixed frame
   *@param [in] rank: Rank of star magnitude in read catalogue
   *@param [in] quaternion_i2b: Quaternion from the inertial frame to the body-fixed frame
   */
  s2e::math::Vector<3> GetStarDirection_b(size_t rank, s2e::math::Quaternion quaternion_i2b) const;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

  bool IsCalcEnabled = true;  //!< Calculation enable flag

 private:
  std::vector<HipparcosData> hipparcos_catalogue_;  //!< Data base of the read Hipparcos catalogue
  double max_magnitude_;                            //!< Maximum magnitude in the data base
  std::string catalogue_path_;                      //!< Path to Hipparcos catalog file
};

/**
 *@fn InitHipparcosCatalogue
 *@brief Initialize function for HipparcosCatalogue class
 *@param [in] file_name: Path to the initialize function
 */
HipparcosCatalogue* InitHipparcosCatalogue(std::string file_name);

} // namespace s2e::environment

#endif  // S2E_ENVIRONMENT_GLOBAL_HIPPARCOS_CATALOGUE_HPP_
