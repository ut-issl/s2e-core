/**
 * @file gpio_port.cpp
 * @brief Class to emulate GPIO(General Purpose Input and Output) port
 */

#include "gpio_port.hpp"

GPIOPort::GPIOPort(int port_id, IGPIOCompo* compo) : kPortId(port_id) {
  hl_state_ = GPIO_LOW;
  component_ = compo;
}

GPIOPort::~GPIOPort() {}

int GPIOPort::DigitalWrite(bool isHigh) {
  if (hl_state_ != isHigh) {
    // Call interaction function when detecting the change of the HIGH/LOW state
    if (component_ != nullptr) {
      component_->GpioStateChanged(kPortId, isHigh);
    }
  }
  hl_state_ = isHigh;
  return 0;
}

bool GPIOPort::DigitalRead() { return hl_state_; }
