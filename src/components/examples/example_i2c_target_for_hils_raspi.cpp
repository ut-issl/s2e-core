/**
 * @file example_i2c_target_for_hils.cpp
 * @brief Example of component emulation for I2C target side communication in HILS environment
 */

#include "example_i2c_target_for_hils_raspi.hpp"

// #include <fcntl.h>
// #include <linux/i2c-dev.h>




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
  memcpy(xfer.txBuf, tlm, 4);

  return;

}

int ExampleI2cTargetForHilsRaspi::getControlBits(int address /* max 127 */, bool open) {
		/*
		Excerpt from http://abyz.me.uk/rpi/pigpio/cif.html#bscXfer regarding the control bits:

		22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
		a	a	a	a	a	a	a	-	-	IT HC TF IR RE TE BK EC ES PL PH I2 SP EN

		Bits 0-13 are copied unchanged to the BSC CR register. See pages 163-165 of the Broadcom 
		peripherals document for full details. 

		aaaaaaa defines the I2C slave address (only relevant in I2C mode)
		IT	invert transmit status flags
		HC	enable host control
		TF	enable test FIFO
		IR	invert receive status flags
		RE	enable receive
		TE	enable transmit
		BK	abort operation and clear FIFOs
		EC	send control register as first I2C byte
		ES	send status register as first I2C byte
		PL	set SPI polarity high
		PH	set SPI phase high
		I2	enable I2C mode
		SP	enable SPI mode
		EN	enable BSC peripheral
		*/

		// Flags like this: 0b/*IT:*/0/*HC:*/0/*TF:*/0/*IR:*/0/*RE:*/0/*TE:*/0/*BK:*/0/*EC:*/0/*ES:*/0/*PL:*/0/*PH:*/0/*I2:*/0/*SP:*/0/*EN:*/0;

		int flags;
		if(open)
				flags = /*RE: */ (1 << 9) | /*TE:*/ (1 << 8) | /*I2:*/ (1 << 2) | /*EN:*/ (1 << 0);
		else // Close/Abort
				flags = /*BK:*/ (1 << 7) | /*I2:*/ (0 << 2) | /*EN:*/ (0 << 0);

		return (address << 16 /*= to the start of significant bits*/) | flags;
}
