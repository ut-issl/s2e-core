/**
 * @file example_serial_communication_with_obc.cpp
 * @brief Example of component emulation with communication between OBC Flight software
 */

#include "example_serial_communication_with_obc.hpp"

#include <string.h>

ExampleSerialCommunicationWithObc::ExampleSerialCommunicationWithObc(ClockGenerator* clock_gen, int port_id, OBC* obc)
    : Component(1000, clock_gen), ObcCommunicationBase(port_id, obc) {
  Initialize();
}
ExampleSerialCommunicationWithObc::ExampleSerialCommunicationWithObc(ClockGenerator* clock_gen, int port_id, int prescaler, OBC* obc)
    : Component(prescaler, clock_gen), ObcCommunicationBase(port_id, obc) {
  Initialize();
}

int ExampleSerialCommunicationWithObc::Initialize() {
  for (int i = 0; i < MAX_MEMORY_LEN; i++) {
    memory.push_back(0);
  }
  return 0;
}

ExampleSerialCommunicationWithObc::~ExampleSerialCommunicationWithObc() {}

int ExampleSerialCommunicationWithObc::ParseCommand(const int cmd_size) {
  if (cmd_size < 4) {
    return -1;
  }
  if (rx_buffer_[0] != 'S' || rx_buffer_[1] != 'E' || rx_buffer_[2] != 'T') {
    return -1;
  }
  memory.pop_back();
  memory.insert(memory.begin(), rx_buffer_[3]);
  memory[MAX_MEMORY_LEN - 1] = '\n';
  return 0;
}
int ExampleSerialCommunicationWithObc::GenerateTelemetry() {
  for (int i = 0; i < MAX_MEMORY_LEN; i++) {
    tx_buff[i] = (unsigned char)memory[i];
  }
  tx_buffer_.assign(std::begin(tx_buff), std::end(tx_buff));
  return sizeof(tx_buff);
}
void ExampleSerialCommunicationWithObc::MainRoutine(int count) {
  UNUSED(count);
  ReceiveCommand(0, 5);
  SendTelemetry(0);
}

void ExampleSerialCommunicationWithObc::GpioStateChanged(int port_id, bool isPosedge) {
  printf("interrupted. portid = %d, isPosedge = %d./n", port_id, isPosedge);
}
