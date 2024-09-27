/**
 * @file example_serial_communication_for_hils.cpp
 * @brief Example of component emulation for UART communication in HILS environment
 */
#include "example_serial_communication_for_hils.hpp"

#include <utilities/macros.hpp>

namespace s2e::components {

ExampleSerialCommunicationForHils::ExampleSerialCommunicationForHils(ClockGenerator* clock_generator, const int sils_port_id, OnBoardComputer* obc,
                                                                     const unsigned int hils_port_id, const unsigned int baud_rate,
                                                                     HilsPortManager* hils_port_manager, const int mode_id)
    : Component(300, clock_generator), UartCommunicationWithObc(sils_port_id, obc, hils_port_id, baud_rate, hils_port_manager), mode_id_(mode_id) {}

ExampleSerialCommunicationForHils::~ExampleSerialCommunicationForHils() {}

int ExampleSerialCommunicationForHils::ParseCommand(const int command_size) {
  UNUSED(command_size);

  if (mode_id_ == 1) {
    for (int i = 0; i < kMemorySize; i++) {
      memory_[i] = (unsigned char)rx_buffer_[i];
    }
    memory_[kMemorySize - 1] = '\0';
  } else if (mode_id_ == 0)  // data returned from the responder component to the sender component
  {
    // the first row will overwrite the progress output in SampleCase.cpp
    std::cout << std::endl << rx_buffer_[0] << rx_buffer_[1] << rx_buffer_[2] << std::endl;
  }
  return 0;
}

int ExampleSerialCommunicationForHils::GenerateTelemetry() {
  if (mode_id_ == 0)  // The sender component sends ABC, BCD, CDE ...
  {
    for (int i = 0; i < kMemorySize; i++) {
      tx_[i] = (char)('A' + counter_ + i);
    }
    counter_++;
    tx_[kMemorySize - 1] = '\0';
    if (counter_ > kNumAlphabet - (kMemorySize - 1)) {
      counter_ = 0;
    }
    tx_buffer_.assign(std::begin(tx_), std::end(tx_));
    return sizeof(tx_);
  } else if (mode_id_ == 1)  // The responder component sends back the received message.
  {
    for (int i = 0; i < kMemorySize; i++) {
      tx_[i] = (unsigned char)memory_[i];
    }
    tx_buffer_.assign(std::begin(tx_), std::end(tx_));
    return sizeof(tx_);
  }
  return 0;
}

void ExampleSerialCommunicationForHils::MainRoutine(const int time_count) {
  UNUSED(time_count);

  ReceiveCommand(0, kMemorySize);
  SendTelemetry(0);
}

}  // namespace s2e::components
