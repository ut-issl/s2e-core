/**
 * @file example_i2c_controller_for_hils.cpp
 * @brief Example of component emulation for I2C controller side communication in HILS environment
 */
#include "example_i2c_controller_for_hils.hpp"

ExampleI2cControllerForHils::ExampleI2cControllerForHils(const int prescaler, ClockGenerator* clock_generator, const unsigned int hils_port_id,
                                                         const unsigned int baud_rate, const unsigned int tx_buffer_size,
                                                         const unsigned int rx_buffer_size, HilsPortManager* hils_port_manager)
    : Component(prescaler, clock_generator), I2cController(hils_port_id, baud_rate, tx_buffer_size, rx_buffer_size, hils_port_manager) {}

ExampleI2cControllerForHils::~ExampleI2cControllerForHils() {}

void ExampleI2cControllerForHils::MainRoutine(int count) {
  UNUSED(count);

  RequestTlm();
  Receive();
  return;
}

void ExampleI2cControllerForHils::RequestTlm() {
  const unsigned char kReqTlmCmdSize = 5;
  unsigned char req_tlm_cmd[kReqTlmCmdSize];
  req_tlm_cmd[0] = kCmdHeader_;
  req_tlm_cmd[1] = kWriteCmd_;
  req_tlm_cmd[2] = 0x01;  // bytes to write
  req_tlm_cmd[3] = 0x00;  // register address
  req_tlm_cmd[kReqTlmCmdSize - 1] = kCmdFooter_;
  tx_buffer_.assign(std::begin(req_tlm_cmd), std::end(req_tlm_cmd));
  SendCommand(kReqTlmCmdSize);
  std::cout << "I2C Controller requests Telemetry" << std::endl;
  return;
}

void ExampleI2cControllerForHils::Receive() {
  const unsigned char kTlmSize = 5;
  const unsigned char kReceiveCmdSize = 4;
  unsigned char receive_cmd[kReceiveCmdSize];
  receive_cmd[0] = kCmdHeader_;
  receive_cmd[1] = kReadCmd_;
  receive_cmd[2] = kTlmSize;  // bytes to read
  receive_cmd[kReceiveCmdSize - 1] = kCmdFooter_;
  tx_buffer_.assign(std::begin(receive_cmd), std::end(receive_cmd));
  SendCommand(kReceiveCmdSize);

  rx_buffer_.resize(kTlmSize);
  int ret = ReceiveTelemetry(kTlmSize);
  if (ret <= 0) return;
  std::cout << "I2C Controller received: ";
  for (int i = 0; i < ret; i++) {
    std::cout << rx_buffer_[i];
  }
  std::cout << std::endl;
  return;
}
