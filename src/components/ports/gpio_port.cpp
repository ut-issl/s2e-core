/**
 * @file gpio_port.cpp
 * @brief Class to emulate GPIO(General Purpose Input and Output) port
 */

#include "gpio_port.hpp"

GPIOPort::GPIOPort(const unsigned int port_id, IGPIOCompo* compo) : kPortId(port_id) {
  high_low_state_ = GPIO_LOW;
  component_ = compo;
}

GPIOPort::~GPIOPort() {}

int GPIOPort::DigitalWrite(const bool isHigh) {
  if (high_low_state_ != isHigh) {
    // Call interaction function when detecting the change of the HIGH/LOW state
    if (component_ != nullptr) {
      component_->GpioStateChanged(kPortId, isHigh);
    }
  }
  high_low_state_ = isHigh;
  return 0;
}

bool GPIOPort::DigitalRead() { return high_low_state_; }
