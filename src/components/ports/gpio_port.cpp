/**
 * @file gpio_port.cpp
 * @brief Class to emulate GPIO(General Purpose Input and Output) port
 */

#include "gpio_port.hpp"

GpioPort::GpioPort(const unsigned int port_id, IGPIOCompo* component) : kPortId(port_id) {
  high_low_state_ = GPIO_LOW;
  component_ = component;
}

GpioPort::~GpioPort() {}

int GpioPort::DigitalWrite(const bool is_high) {
  if (high_low_state_ != is_high) {
    // Call interaction function when detecting the change of the HIGH/LOW state
    if (component_ != nullptr) {
      component_->GpioStateChanged(kPortId, is_high);
    }
  }
  high_low_state_ = is_high;
  return 0;
}

bool GpioPort::DigitalRead() { return high_low_state_; }
