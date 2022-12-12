/**
 * @file COSMOSWrapper.h
 * @brief Wrapper for COSMOS
 * @details COSMOS (Currently, called OpenC3) : https://openc3.com/
 * @note This file is very old and recently not managed well...
 */

#pragma once
#include <string>

#include "COSMOS_TCP_IF.h"
/**
 * @class COSMOSWrapper
 * @brief Wrapper for COSMOS
 */
class COSMOSWrapper : private COSMOS_TCP_IF {
 public:
  /**
   * @fn COSMOSWrapper
   * @brief Constructor.
   */
  COSMOSWrapper(bool enable);
  /**
   * @fn COSMOSWrapper
   * @brief Constructor.
   */
  COSMOSWrapper(unsigned short port_num, bool enable, char* server_ip);
  /**
   * @fn COSMOSWrapper
   * @brief Default Constructor. When not in use
   */
  COSMOSWrapper();
  /**
   * @fn ~COSMOSWrapper
   * @brief Destructor.
   */
  ~COSMOSWrapper();

  /**
   * @fn Cmd
   * @brief Wrapper of cmd function to send data from COSMOS to RDP
   * @param [in] cmd_str: String of command
   */
  void Cmd(std::string cmd_str);
  /**
   * @fn Cmd_RDP_SAT_CMD_EXTERNAL_TORQUE
   * @brief Command to set external torque
   */
  void Cmd_RDP_SAT_CMD_EXTERNAL_TORQUE(uint8_t torque_frame, uint32_t duration_ms, float ext_torque_x, float ext_torque_y, float ext_torque_z);

  /**
   * @fn Initialize
   * @brief Initialize the TCP connection with COSMOS
   */
  void Initialize();
  /**
   * @fn Finalize
   * @brief Finalize the TCP connection with COSMOS
   */
  void Finalize();
  /**
   * @fn Enable
   * @brief Return enable flag
   */
  bool Enable() const;

 private:
  /**
   * @fn CosmosEval
   * @brief Send string to COSMOS
   */
  void CosmosEval(std::string expr);

  const bool enable_;  //!< Enable flag
};
