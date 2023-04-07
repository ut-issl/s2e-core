/**
 * @file relative_information.hpp
 * @brief Base class to manage relative information between spacecraft
 */

#ifndef S2E_MULTIPLE_SPACECRAFT_RELATIVE_INFORMATION_HPP_
#define S2E_MULTIPLE_SPACECRAFT_RELATIVE_INFORMATION_HPP_

#include <string>

#include "../../dynamics/dynamics.hpp"
#include "../../library/logger/loggable.hpp"
#include "../../library/logger/logger.hpp"

/**
 * @class RelativeInformation
 * @brief Base class to manage relative information between spacecraft
 */
class RelativeInformation : public ILoggable {
 public:
  /**
   * @fn RelativeInformation
   * @brief Constructor
   */
  RelativeInformation();
  /**
   * @fn ~RelativeInformation
   * @brief Destructor
   */
  ~RelativeInformation();

  /**
   * @fn Update
   * @brief Update all relative information
   */
  void Update();
  /**
   * @fn RegisterDynamicsInfo
   * @brief Register dynamics information of target spacecraft
   * @param [in] spacecraft_id: ID of target spacecraft
   * @param [in] dynamics: Dynamics information of the target spacecraft
   */
  void RegisterDynamicsInfo(const int spacecraft_id, const Dynamics* dynamics);
  /**
   * @fn RegisterDynamicsInfo
   * @brief Remove dynamics information of target spacecraft
   * @param [in] spacecraft_id: ID of target spacecraft
   */
  void RemoveDynamicsInfo(const int spacecraft_id);

  // Override classes for ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override function of GetLogHeader
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override function of GetLogValue
   */
  virtual std::string GetLogValue() const;

  /**
   * @fn LogSetup
   * @brief Logging setup for relative information
   */
  void LogSetup(Logger& logger);

  // Getter
  /**
   * @fn GetRelativeAttitudeQuaternion
   * @brief Return relative attitude quaternion of the target spacecraft with respect to the reference spacecraft
   * @param [in] target_spacecraft_id: ID of target spacecraft
   * @param [in] reference_spacecraft_id: ID of reference spacecraft
   */
  inline libra::Quaternion GetRelativeAttitudeQuaternion(const int target_spacecraft_id, const int reference_spacecraft_id) const {
    return relative_attitude_quaternion_list_[target_spacecraft_id][reference_spacecraft_id];
  }
  /**
   * @fn GetRelativePosition_i_m
   * @brief Return relative position of the target spacecraft with respect to the reference spacecraft in the inertial frame and unit [m]
   * @param [in] target_spacecraft_id: ID of target spacecraft
   * @param [in] reference_spacecraft_id: ID of reference spacecraft
   */
  inline libra::Vector<3> GetRelativePosition_i_m(const int target_spacecraft_id, const int reference_spacecraft_id) const {
    return relative_position_list_i_m_[target_spacecraft_id][reference_spacecraft_id];
  }
  /**
   * @fn GetRelativeVelocity_i_m
   * @brief Return relative velocity of the target spacecraft with respect to the reference spacecraft in the inertial frame and unit [m]
   * @param [in] target_spacecraft_id: ID of target spacecraft
   * @param [in] reference_spacecraft_id: ID of reference spacecraft
   */
  inline libra::Vector<3> GetRelativeVelocity_i_m_s(const int target_spacecraft_id, const int reference_spacecraft_id) const {
    return relative_velocity_list_i_m_s_[target_spacecraft_id][reference_spacecraft_id];
  }
  /**
   * @fn GetRelativeDistance_m
   * @brief Return relative distance between the target spacecraft and the reference spacecraft in unit [m]
   * @param [in] target_spacecraft_id: ID of target spacecraft
   * @param [in] reference_spacecraft_id: ID of reference spacecraft
   */
  inline double GetRelativeDistance_m(const int target_spacecraft_id, const int reference_spacecraft_id) const {
    return relative_distance_list_m_[target_spacecraft_id][reference_spacecraft_id];
  };
  /**
   * @fn GetRelativePosition_rtn_m
   * @brief Return relative position of the target spacecraft with respect to the reference spacecraft in the RTN frame of the reference spacecraft
   * @param [in] target_spacecraft_id: ID of target spacecraft
   * @param [in] reference_spacecraft_id: ID of reference spacecraft
   */
  inline libra::Vector<3> GetRelativePosition_rtn_m(const int target_spacecraft_id, const int reference_spacecraft_id) const {
    return relative_position_list_rtn_m_[target_spacecraft_id][reference_spacecraft_id];
  }
  /**
   * @fn GetRelativeVelocity_rtn_m_s
   * @brief Return relative velocity of the target spacecraft with respect to the reference spacecraft in the RTN frame of the reference spacecraft
   * @param [in] target_spacecraft_id: ID of target spacecraft
   * @param [in] reference_spacecraft_id: ID of reference spacecraft
   */
  inline libra::Vector<3> GetRelativeVelocity_rtn_m_s(const int target_spacecraft_id, const int reference_spacecraft_id) const {
    return relative_velocity_list_rtn_m_s_[target_spacecraft_id][reference_spacecraft_id];
  }

  /**
   * @fn GetReferenceSatDynamics
   * @brief Return the dynamics information of a spacecraft
   * @param [in] target_spacecraft_id: ID of the spacecraft
   */
  inline const Dynamics* GetReferenceSatDynamics(const int reference_spacecraft_id) const { return dynamics_database_.at(reference_spacecraft_id); };

 private:
  std::map<const int, const Dynamics*> dynamics_database_;  //!< Dynamics database of all spacecraft

  std::vector<std::vector<libra::Vector<3>>> relative_position_list_i_m_;          //!< Relative position list in the inertial frame in unit [m]
  std::vector<std::vector<libra::Vector<3>>> relative_velocity_list_i_m_s_;        //!< Relative velocity list in the inertial frame in unit [m/s]
  std::vector<std::vector<libra::Vector<3>>> relative_position_list_rtn_m_;        //!< Relative position list in the RTN frame in unit [m]
  std::vector<std::vector<libra::Vector<3>>> relative_velocity_list_rtn_m_s_;      //!< Relative velocity list in the RTN frame in unit [m/s]
  std::vector<std::vector<double>> relative_distance_list_m_;                      //!< Relative distance list in unit [m]
  std::vector<std::vector<libra::Quaternion>> relative_attitude_quaternion_list_;  //!< Relative attitude quaternion list

  /**
   * @fn CalcRelativeAttitudeQuaternion
   * @brief Calculate and return the relative attitude quaternion
   * @param [in] target_spacecraft_id: ID of the spacecraft
   * @param [in] reference_spacecraft_id: ID of reference spacecraft
   */
  libra::Quaternion CalcRelativeAttitudeQuaternion(const int target_spacecraft_id, const int reference_spacecraft_id);
  /**
   * @fn CalcRelativePosition_rtn_m
   * @brief Calculate and return the relative position in RTN frame
   * @param [in] target_spacecraft_id: ID of the spacecraft
   * @param [in] reference_spacecraft_id: ID of reference spacecraft
   */
  libra::Vector<3> CalcRelativePosition_rtn_m(const int target_spacecraft_id, const int reference_spacecraft_id);
  /**
   * @fn CalcRelativeVelocity_rtn_m_s
   * @brief Calculate and return the relative velocity in RTN frame
   * @param [in] target_spacecraft_id: ID of the spacecraft
   * @param [in] reference_spacecraft_id: ID of reference spacecraft
   */
  libra::Vector<3> CalcRelativeVelocity_rtn_m_s(const int target_spacecraft_id, const int reference_spacecraft_id);

  /**
   * @fn ResizeLists
   * @brief Resize list suit with the dynamics database
   */
  void ResizeLists();
};

#endif  // S2E_MULTIPLE_SPACECRAFT_RELATIVE_INFORMATION_HPP_
