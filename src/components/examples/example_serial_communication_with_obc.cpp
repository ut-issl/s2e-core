/**
 * @file example_serial_communication_with_obc.cpp
 * @brief Example of component emulation with communication between OBC flight software
 */

#include "example_serial_communication_with_obc.hpp"

#include <string.h>

ExampleSerialCommunicationWithObc::ExampleSerialCommunicationWithObc(ClockGenerator* clock_generator, int port_id, OnBoardComputer* obc)
    : Component(1000, clock_generator), UartCommunicationWithObc(port_id, obc) {
  Initialize();
}
ExampleSerialCommunicationWithObc::ExampleSerialCommunicationWithObc(ClockGenerator* clock_generator, int port_id, int prescaler,
                                                                     OnBoardComputer* obc)
    : Component(prescaler, clock_generator), UartCommunicationWithObc(port_id, obc) {
  Initialize();
}

int ExampleSerialCommunicationWithObc::Initialize() {
  for (int i = 0; i < kMaxMemoryLength; i++) {
    memory_.push_back(0);
  }
  return 0;
}

ExampleSerialCommunicationWithObc::~ExampleSerialCommunicationWithObc() {}

int ExampleSerialCommunicationWithObc::ParseCommand(const int command_size) {
  if (command_size < 4) {
    return -1;
  }
  if (rx_buffer_[0] != 'S' || rx_buffer_[1] != 'E' || rx_buffer_[2] != 'T') {
    return -1;
  }
  memory_.pop_back();
  memory_.insert(memory_.begin(), rx_buffer_[3]);
  memory_[kMaxMemoryLength - 1] = '\n';
  return 0;
}
int ExampleSerialCommunicationWithObc::GenerateTelemetry() {
  for (int i = 0; i < kMaxMemoryLength; i++) {
    tx_buffer_[i] = (unsigned char)memory_[i];
  }
  tx_buffer_.assign(std::begin(tx_buffer_), std::end(tx_buffer_));
  return sizeof(tx_buffer_);
}
void ExampleSerialCommunicationWithObc::MainRoutine(const int time_count) {
  UNUSED(time_count);
  ReceiveCommand(0, 5);
  SendTelemetry(0);
}

void ExampleSerialCommunicationWithObc::GpioStateChanged(int port_id, bool isPosedge) {
  printf("interrupted. portid = %d, isPosedge = %d./n", port_id, isPosedge);
}
