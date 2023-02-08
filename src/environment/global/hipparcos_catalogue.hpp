/**
 *@file hipparcos_catalogue.hpp
 *@brief Class to calculate star direction with Hipparcos catalogue
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_HIPPAROCOS_CATALOGUE_H_
#define S2E_ENVIRONMENT_GLOBAL_HIPPAROCOS_CATALOGUE_H_

#include <library/math/quaternion.hpp>
#include <library/math/Vector.hpp>
#include <interface/log_output/loggable.hpp>
#include <vector>

/**
 *@struct HipData
 *@brief Hipparcos catalogue data
 */
struct HipData {
  int hip_num;  //!< Hipparcos number
  double vmag;  //!< Visible magnitude
  double ra;    //!< Right ascention [rad]
  double de;    //!< Declination [rad]
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
  bool ReadContents(const std::string& filename, const char delimiter);

  /**
   *@fn GetCatalogueSize
   *@brief Return read catalogue size
   */
  int GetCatalogueSize() const { return hip_catalogue.size(); }
  /**
   *@fn GetHipID
   *@brief Return Hipparcos ID of a star
   *@param [in] rank: Rank of star magnitude in read catalogue
   */
  int GetHipID(int rank) const { return hip_catalogue[rank].hip_num; }
  /**
   *@fn GetVmag
   *@brief Return magnitude in visible wave length of a star
   *@param [in] rank: Rank of star magnitude in read catalogue
   */
  double GetVmag(int rank) const { return hip_catalogue[rank].vmag; }
  /**
   *@fn GetRA
   *@brief Return right ascension of a star
   *@param [in] rank: Rank of star magnitude in read catalogue
   */
  double GetRA(int rank) const { return hip_catalogue[rank].ra; }
  /**
   *@fn GetDE
   *@brief Return declination of a star
   *@param [in] rank: Rank of star magnitude in read catalogue
   */
  double GetDE(int rank) const { return hip_catalogue[rank].de; }
  /**
   *@fn GetStarDir_i
   *@brief Return direction vector of a star in the inertial frame
   *@param [in] rank: Rank of star magnitude in read catalogue
   */
  libra::Vector<3> GetStarDir_i(int rank) const;
  /**
   *@fn GetStarDir_b
   *@brief Return direction vector of a star in the body-fixed frame
   *@param [in] rank: Rank of star magnitude in read catalogue
   *@param [in] rank: Quaternion from the inertial frame to the body-fixed frame
   */
  libra::Vector<3> GetStarDir_b(int rank, Quaternion q_i2b) const;

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
  std::vector<HipData> hip_catalogue;  //!< Data base of the read Hipparcos catalogue
  double max_magnitude_;               //!< Maximum magnitude in the data base
  std::string catalogue_path_;         //!< Path to Hipparcos catalog file
};

#endif  // S2E_ENVIRONMENT_GLOBAL_HIPPAROCOS_CATALOGUE_H_
