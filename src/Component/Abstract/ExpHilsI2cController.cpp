#include "ExpHilsI2cController.h"

ExpHilsI2cController::ExpHilsI2cController(const int prescaler,
                                           ClockGenerator* clock_gen,
                                           const unsigned int hils_port_id,
                                           const unsigned int baud_rate,
                                           const unsigned int tx_buf_size,
                                           const unsigned int rx_buf_size,
                                           HilsPortManager* hils_port_manager)
    : ComponentBase(prescaler, clock_gen),
      I2cControllerCommunicationBase(hils_port_id, baud_rate, tx_buf_size,
                                     rx_buf_size, hils_port_manager) {}

ExpHilsI2cController::~ExpHilsI2cController() {}

void ExpHilsI2cController::MainRoutine(int count) {
  RequestTlm();
  Receive();
  return;
}

void ExpHilsI2cController::RequestTlm() {
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

void ExpHilsI2cController::Receive() {
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
