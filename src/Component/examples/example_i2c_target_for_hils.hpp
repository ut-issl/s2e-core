/**
 * @file example_i2c_target_for_hils.hpp
 * @brief Example of component emulation for I2C target side communication in HILS environment
 */

#pragma once
#include <vector>

#include "../Abstract/ComponentBase.h"
#include "../Abstract/ObcI2cTargetCommunicationBase.h"

/**
 * @class ExpHilsI2cTarget
 * @brief Example of component emulation for I2C target side communication in HILS environment
 * @details Supposed to be used in connection with the following I2C-USB Controller converter
 *          MFT200XD Data Sheet:https://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT200XD.pdf
 *          Telemetry size = 5 bytes(ASCII)
 *          Telemetry changes; ABCDE, BCDEF, ..., VWXYZ, ABCDE, ...
 */
class ExpHilsI2cTarget : public ComponentBase, public ObcI2cTargetCommunicationBase {
 public:
  /**
   * @fn ExpHilsI2cTarget
   * @brief Constructor
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] sils_port_id: Port ID for communication line b/w OBC
   * @param [in] i2c_address: I2C address of the target device (This value should be compatible with MFT200XD's setting)
   * @param [in] obc: The communication target OBC
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] hils_port_manager: HILS port manager
   */
  ExpHilsI2cTarget(const int prescaler, ClockGenerator* clock_gen, const int sils_port_id, unsigned char i2c_address, OBC* obc,
                   const unsigned int hils_port_id, HilsPortManager* hils_port_manager);
  /**
   * @fn ~ExpHilsI2cTarget
   * @brief Destructor
   */
  ~ExpHilsI2cTarget();

 protected:
  // Override functions for ComponentBase
  /**
   * @fn MainRoutine
   * @brief Main routine to receive command and send telemetry
   */
  void MainRoutine(int count);

 private:
  unsigned char tlm_counter_ = 0;           //!< Telemetry counter
  const unsigned int kStoredFrameSize = 3;  //!< Frame size
  const unsigned char kNumAlphabet = 26;    //!< Number of alphabet
};
