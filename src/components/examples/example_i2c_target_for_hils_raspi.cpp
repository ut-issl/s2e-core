/**
 * @file example_i2c_target_for_hils.cpp
 * @brief Example of component emulation for I2C target side communication in HILS environment
 */

#include "example_i2c_target_for_hils_raspi.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <pigpio.h>
#include <unistd.h>



ExampleI2cTargetForHilsRaspi::ExampleI2cTargetForHilsRaspi(const int prescaler, ClockGenerator* clock_generator, const int sils_port_id,
                                                           unsigned char i2c_address, OnBoardComputer* obc, const unsigned int hils_port_id,
                                                           HilsPortManager* hils_port_manager)
    : Component(prescaler, clock_generator), I2cTargetCommunicationWithObc(sils_port_id, hils_port_id, i2c_address, obc, hils_port_manager) {



  gpioInitialise();
  std::cout<<"Initialized GPIOs"<<std::endl;
  // Transmit Hello World to the I2C
  xfer.control = getControlBits(slaveAddress, true);
  // memcpy(xfer.txBuf, "Hello World", 11);
  xfer.txCnt = 0;
}

ExampleI2cTargetForHilsRaspi::~ExampleI2cTargetForHilsRaspi() {
    // Close old device (if any)
  xfer.control = getControlBits(slaveAddress, false);  // To avoid conflicts when restarting
  bscXfer(&xfer);
    std::cout<<"I2C Desabled"<<std::endl;
}

void ExampleI2cTargetForHilsRaspi::MainRoutine(const int time_count) {
  UNUSED(time_count);

  // update telemetry data
  const unsigned char kTlmSize = 5;
  unsigned char tlm[kTlmSize] = {0};
  for (unsigned char i = 0; i < kTlmSize; i++) {
    tlm[i] = 'A' + tlm_counter_ + i;
  }
  WriteRegister(0, tlm, kTlmSize);
  tlm_counter_++;
  if (tlm_counter_ > kNumAlphabet - kTlmSize) {
    tlm_counter_ = 0;  // ABCDE, ..., VWXYZ, ABCDE, ...
  }

  ReceiveCommand();
  int additional_frame_num = kStoredFrameSize - GetStoredFrameCounter();
  if (additional_frame_num <= 0) return;

  // store telemetry in converter up to kStoredFrameSize
  for (int i = 0; i < additional_frame_num; i++) {
    SendTelemetry(kTlmSize);
    std::cout << "I2C Target sends to converter: " << tlm[0] << tlm[1] << tlm[2] << tlm[3] << tlm[4] << std::endl;
  }

  return;

}

  
  int getControlBits(int, bool);
  const int slaveAddress = 0x10;  // <-- Your address of choice
  bsc_xfer_t xfer;                // Struct to control data flow
  int command = 0;
  char hello[] = "Hello World";
  int nTransferred, nRxFIFO, nTxFIFO;