#include "ExpHilsI2cController.h"

ExpHilsI2cController::ExpHilsI2cController(
  ClockGenerator* clock_gen,
  const unsigned int hils_port_id,
  HilsPortManager* hils_port_manager)
  : ComponentBase(0, clock_gen),
  ObcI2cControllerCommunicationBase(hils_port_id, 9600, 256, 256, hils_port_manager)
{
}

ExpHilsI2cController::~ExpHilsI2cController()
{
}

void ExpHilsI2cController::MainRoutine(int count)
{
  counter_++;
  if (counter_ % 4 == 0)
  {
    const unsigned char clear_buffer_cmd_size = 5;
    unsigned char clear_buffer_tx[clear_buffer_cmd_size];
    clear_buffer_tx[0] = kHeader_;
    clear_buffer_tx[1] = 0x00; // General Call Address
    clear_buffer_tx[2] = 0x01; // data_size
    clear_buffer_tx[3] = 0x06; // clear cmd
    clear_buffer_tx[clear_buffer_cmd_size - 1] = kFooter_;
    tx_buffer_.assign(std::begin(clear_buffer_tx), std::end(clear_buffer_tx));
    SendCommand(clear_buffer_cmd_size); // Clear buffer
    std::cout << "Clear Buffer" << std::endl;
    return;
  }
  else if (counter_ % 4 == 1)
  {
    const unsigned char req_tlm_cmd_size = 5;
    unsigned char req_tlm_tx[req_tlm_cmd_size];
    req_tlm_tx[0] = kHeader_;
    req_tlm_tx[1] = kWrite_;
    req_tlm_tx[2] = 0x01; // data_size
    req_tlm_tx[3] = 0x00; // reg_addr
    req_tlm_tx[req_tlm_cmd_size - 1] = kFooter_;
    tx_buffer_.assign(std::begin(req_tlm_tx), std::end(req_tlm_tx));
    SendCommand(req_tlm_cmd_size); // Request Tlm
    std::cout << "Request Tlm" << std::endl;
  //  return;
  //}
  //else if (counter_ % 4 == 2)
  //{
    const unsigned char read_cmd_size = 4;
    unsigned char read_tx[read_cmd_size];
    read_tx[0] = kHeader_;
    read_tx[1] = kRead_;
    read_tx[2] = 0x05; // tlm_size
    read_tx[read_cmd_size - 1] = kFooter_;
    tx_buffer_.assign(std::begin(read_tx), std::end(read_tx));
    SendCommand(read_cmd_size); // Read
    std::cout << "Read Buffer" << std::endl;
    return;
  }
  else if(counter_ % 4 == 3)
  {
    unsigned char tlm_size = 5;
    rx_buffer_.resize(tlm_size);
    int ret = ReceiveTelemetry(tlm_size);
    std::cout << "Controller Received Bytes: " << ret << std::endl;
    if (ret <= 0) return;
    std::cout << "Controller Received: ";
    for (int i = 0; i < ret; i++)
    {
      std::cout << rx_buffer_[i];
    }
    std::cout << std::endl;
    return;
  }
}
