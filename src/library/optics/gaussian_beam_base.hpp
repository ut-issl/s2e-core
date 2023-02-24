/**
 * @file gaussian_beam_base.hpp
 * @brief Class to express gaussian beam laser
 */

#ifndef S2E_LIBRARY_OPTICS_GAUSSIAN_BEAM_BASE_HPP_
#define S2E_LIBRARY_OPTICS_GAUSSIAN_BEAM_BASE_HPP_

#include "../math/vector.hpp"

/**
 * @class GaussianBeamBase
 * @brief Class to express gaussian beam laser
 */
class GaussianBeamBase {
 public:
  /**
   * @fn GaussianBeamBase
   * @brief Constructor
   * @param [in] wavelength_m: Wavelength [m]
   * @param [in] radius_beam_waist_m: Radius of beam waist [m]
   * @param [in] total_power_W: Total power [W]
   */
  GaussianBeamBase(double wavelength_m, double radius_beam_waist_m, double total_power_W);
  /**
   * @fn ~GaussianBeamBase
   * @brief Destructor
   */
  ~GaussianBeamBase();

  // Setter
  /**
   * @fn SetWaveLength
   * @brief Set wavelength [m]
   */
  void SetWaveLength(const double wavelength_m);
  /**
   * @fn SetBeamWaistRadius
   * @brief Set radius of beam waist [m]
   */
  void SetBeamWaistRadius(const double radius_beam_waist_m);
  /**
   * @fn SetTotalPower
   * @brief Set total power [W]
   */
  void SetTotalPower(const double total_power_W);
  /**
   * @fn SetPointingVector_i
   * @brief Set pointing direction vector in the inertial frame
   */
  void SetPointingVector_i(const libra::Vector<3> pointing_vector_i);
  /**
   * @fn SetBeamWaistPos_i
   * @brief Set position of beam waist in the inertial frame [m] (Not used?)
   */
  void SetBeamWaistPos_i(const libra::Vector<3> position_beam_waist_i_m);

  // Getter
  /**
   * @fn GetWaveLength
   * @brief Return wavelength [m]
   */
  inline double GetWaveLength() const { return wavelength_m_; }
  /**
   * @fn GetBeamWaistRadius
   * @brief Return radius of beam waist [m]
   */
  inline double GetBeamWaistRadius() const { return radius_beam_waist_m_; }
  /**
   * @fn GetTotalPower
   * @brief Return total power [W]
   */
  inline double GetTotalPower() const { return total_power_W_; }
  /**
   * @fn GetPointingVector_i
   * @brief Return pointing direction vector in the inertial frame
   */
  inline const libra::Vector<3> GetPointingVector_i() const { return pointing_vector_i_; }
  /**
   * @fn GetBeamWaistPos_i
   * @brief Return position of beam waist in the inertial frame [m] (Not used?)
   */
  inline const libra::Vector<3> GetBeamWaistPos_i() const { return position_beam_waist_i_m_; }

  // Calculate functions
  /**
   * @fn CalcBeamWidthRadius
   * @brief
   * @param [in] distance_from_beam_waist_m: Distance from beam waist [m]
   * @return Beam width radius
   */
  double CalcBeamWidthRadius(double distance_from_beam_waist_m);
  /**
   * @fn CalcIntensity
   * @brief
   * @param [in] distance_from_beam_waist_m: Distance from beam waist [m]
   * @param [in] deviation_from_optical_axis_m: Deviation from optical axis [m]
   * @return Intensity
   */
  double CalcIntensity(double distance_from_beam_waist_m, double deviation_from_optical_axis_m);

 private:
  double wavelength_m_;                            //!< Wavelength [m]
  double radius_beam_waist_m_;                     //!< Radius of beam waist [m]
  double total_power_W_;                           //!< Total power [W]
  libra::Vector<3> pointing_vector_i_{0.0};        //!< Pointing direction vector in the inertial frame
  libra::Vector<3> position_beam_waist_i_m_{0.0};  //!< Position of beam waist in the inertial frame [m] (Not used?)
};

#endif  // S2E_LIBRARY_OPTICS_GAUSSIAN_BEAM_BASE_HPP_
