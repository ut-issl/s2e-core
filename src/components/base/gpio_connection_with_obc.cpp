/**
 * @file gpio_connection_with_obc.cpp
 * @brief Base class for GPIO communication with OBC flight software
 *        TODO: consider relation with IGPIOCompo
 */

#include "gpio_connection_with_obc.hpp"

ObcGpioBase::ObcGpioBase(const std::vector<int> port_id, OBC* obc) : port_id_(port_id), obc_(obc) {
  for (size_t i = 0; i < port_id_.size(); i++) {
    obc_->GpioConnectPort(port_id_[i]);
  }
}

ObcGpioBase::~ObcGpioBase() {}

bool ObcGpioBase::Read(const int index) { return obc_->GpioComponentRead(port_id_[index]); }

void ObcGpioBase::Write(const int index, const bool is_high) { obc_->GpioComponentWrite(port_id_[index], is_high); }
