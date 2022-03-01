#include "ExpHilsI2cController.h"

ExpHilsI2cController::ExpHilsI2cController(
  ClockGenerator* clock_gen,
  const unsigned int hils_port_id,
  const unsigned int baud_rate,
  HilsPortManager* hils_port_manager)
  : ComponentBase(30, clock_gen),
  ObcI2cControllerCommunicationBase(hils_port_id, baud_rate, 256, 256, hils_port_manager)
{
}

ExpHilsI2cController::~ExpHilsI2cController()
{
}

void ExpHilsI2cController::MainRoutine(int count)
{
  RequestTlm();
  Receive();
  return;
}

void ExpHilsI2cController::RequestTlm()
{
  const unsigned char req_tlm_cmd_size = 5;
  unsigned char req_tlm_cmd[req_tlm_cmd_size];
  req_tlm_cmd[0] = kHeader_;
  req_tlm_cmd[1] = kWrite_;
  req_tlm_cmd[2] = 0x01; // bytes to write
  req_tlm_cmd[3] = 0x00; // register address
  req_tlm_cmd[req_tlm_cmd_size - 1] = kFooter_;
  tx_buffer_.assign(std::begin(req_tlm_cmd), std::end(req_tlm_cmd));
  SendCommand(req_tlm_cmd_size);
  std::cout << "Controller requests Telemetry" << std::endl;
  return;
}

void ExpHilsI2cController::Receive()
{
  const unsigned int kTlmSize = 5;
  const unsigned char receive_cmd_size = 4;
  unsigned char receive_cmd[receive_cmd_size];
  receive_cmd[0] = kHeader_;
  receive_cmd[1] = kRead_;
  receive_cmd[2] = (char)kTlmSize; // bytes to read
  receive_cmd[receive_cmd_size - 1] = kFooter_;
  tx_buffer_.assign(std::begin(receive_cmd), std::end(receive_cmd));
  SendCommand(receive_cmd_size);

  rx_buffer_.resize(kTlmSize);
  int ret = ReceiveTelemetry(kTlmSize);
  if (ret <= 0) return;
  std::cout << "Controller received: ";
  for (int i = 0; i < ret; i++)
  {
    std::cout << rx_buffer_[i];
  }
  std::cout << std::endl;
  return;
}
